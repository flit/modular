#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import sys
import os
import locale
import curses
import traceback
import logging
from time import time
import pyOCD
from pyOCD.pyDAPAccess import DAPAccess
from pyOCD.pyDAPAccess.cmsis_dap_core import DAP_SWO_STATUS
from pyOCD.coresight.dwt import DWT
from pyOCD.coresight.itm import ITM
from pyOCD.coresight.tpiu import TPIU
from pyOCD.trace.swo import SWOParser
import pyOCD.trace.sink
import pyOCD.debug.elf.symbols
from pyOCD.rtos.argon import ArgonTraceEvent

def dumpHexData(f, data, startAddress=0, width=8):
    i = 0
    while i < len(data):
        f.write("%08x: " % (startAddress + (i * (width / 8))),)

        while i < len(data):
            d = data[i]
            i += 1
            if width == 8:
                f.write("%02x" % d,)
                if i % 4 == 0:
                    f.write("",)
                if i % 16 == 0:
                    break
            elif width == 16:
                f.write("%04x" % d,)
                if i % 8 == 0:
                    break
            elif width == 32:
                f.write("%08x" % d,)
                if i % 4 == 0:
                    break
        f.write('\n')

def format_swo_status(status):
    return ('aA'[int((status & DAP_SWO_STATUS.CAPTURE) != 0)] +
        'eE'[int((status & DAP_SWO_STATUS.ERROR) != 0)] +
        'oO'[int((status & DAP_SWO_STATUS.OVERRUN) != 0)]
        )

SPINNER = ('|', '/', '-', '\\')

SWO_CLK = 4000000
SYS_CLK = 120000000

SWO_DATA_FILENAME = 'swo.dat'
SWO_LOG_FILENAME = 'swo.txt'

WRITE_DATA = False
WRITE_LOG = False

THREAD_NAMES = {
        1 : "adc",
        2 : "init",
        3 : "idle",
        4 : "reader",
        5 : "audio",
        6 : "ui",
        7 : "card",
    }

class SWOFileEventSink(pyOCD.trace.sink.TraceEventSink):
    def __init__(self, f):
        self._file = f
        
    def receive(self, event):
        self._file.write(str(event) + '\n')

class SamplbaerBufferedTimeEventSink(pyOCD.trace.sink.TraceEventSink):
    BUFFERED_TIME_CHANNEL = 1
    BUFFER_COUNT_CHANNEL = 2
    READER_QUEUE_CHANNEL = 3
    THREAD_LOAD_CHANNEL = 31
    
    VALID_CHANNELS = (BUFFERED_TIME_CHANNEL, BUFFER_COUNT_CHANNEL, READER_QUEUE_CHANNEL, THREAD_LOAD_CHANNEL)
    
    def __init__(self, handler):
        self.buffered = [0] * 4
        self.playing = [0] * 4
        self.counts = [(0, 0)] * 4
        self.queued = 0
        self.loads = {}
        self._handler = handler
        
    def receive(self, event):
        if not isinstance(event, pyOCD.trace.events.TraceITMEvent):
            return
        if event.port not in self.VALID_CHANNELS:
            return
        
        data = event.data

        if event.port == self.BUFFERED_TIME_CHANNEL:
            channel = data >> 30
            playing = (data >> 29) & 1
            buffered = data & 0x1fffffff
        
            if not (0 <= channel < 4):
                return
        
            self.buffered[channel] = buffered
            self.playing[channel] = playing
            
            self._handler.bufferedTimeUpdated(self, channel)
        elif event.port == self.BUFFER_COUNT_CHANNEL:
            channel = data >> 14
            freeCount = (data >> 7) & 0x7f
            readyCount = data & 0x7f
        
            if not (0 <= channel < 4):
                return
            
            self.counts[channel] = (freeCount, readyCount)
            
            self._handler.bufferCountUpdated(self, channel)
        elif event.port == self.READER_QUEUE_CHANNEL:
            if data > 32:
                return
            self.queued = data
            self._handler.readerQueueUpdated(self)
        elif event.port == self.THREAD_LOAD_CHANNEL:
            traceEvent = data >> 24
            threadId = (data >> 10) & 0x3fff
            threadLoadPerMille = data & 0x3ff
            
            if traceEvent != 4: # load trace event
                return
            if threadId > 10:
                return
            if threadLoadPerMille > 1000:
                return
            
            self.loads[threadId] = threadLoadPerMille
            
            self._handler.threadLoadUpdated(self, threadId)
            

class ScreenUpdateHandler(object):
    def __init__(self, stdscr):
        self.stdscr = stdscr
    
    def graph_str(self, pct, segments):
#                     blocks = (pct ** 0.3) * segments
        blocks = pct * segments
        filled = min(int(blocks), segments)
        unfilled = segments - filled
        return "▒" * filled + " " * unfilled

    def time_to_graph(self, microsecs):
        MAX_BUF_US = 3.0 * 16.0 * 1024.0 / 48000.0 * 1000000.0
        GRAPH_LEN = 60
        pct = microsecs / MAX_BUF_US
        return self.graph_str(pct, GRAPH_LEN)

    def play_state(self, playing):
        return "SP"[playing]

    def bufferedTimeUpdated(self, sink, channel):
        s = "Channel {}: {} [{}] {:8.1f} ms".format(
                    channel,
                    self.play_state(sink.playing[channel]),
                    self.time_to_graph(sink.buffered[channel]),
                    sink.buffered[channel] / 1000.0)
        self.stdscr.addstr(2 + channel, 0, s)
        self.stdscr.refresh()
    
    def bufferCountUpdated(self, sink, channel):
        freeCount, readyCount = sink.counts[channel]
        s = "Channel {}: R=[{}]".format(
                    channel,
                    self.graph_str(readyCount / 8.0, 8))
        self.stdscr.addstr(7 + channel * 2, 0, s)
        s = "F=[{}]".format(
                    self.graph_str(freeCount / 8.0, 8))
        self.stdscr.addstr(8 + channel * 2, 11, s)
        self.stdscr.refresh()
    
    def readerQueueUpdated(self, sink):
        s = "Read queue: [{}] {:4}".format(
                    self.graph_str(sink.queued / 16.0, 16),
                    sink.queued)
        self.stdscr.addstr(16, 0, s)
        self.stdscr.refresh()
    
    def threadLoadUpdated(self, sink, threadId):
        load = sink.loads[threadId] / 10.0
        s = "Thread #{:2} {:6}: {:8.1f}%".format(threadId, THREAD_NAMES.get(threadId, "???"), load)
        self.stdscr.addstr(17 + threadId, 0, s)
        self.stdscr.refresh()

def do_it(stdscr):
    curses.curs_set(0)
    
    brd = None
    try:
        brd = pyOCD.board.MbedBoard.chooseBoard(board_id='02250000501200334e453571ffffffff001dffff97969906', target_override='k22fa12', init_board=False)
        brd.target.setHaltOnConnect(False)
        brd.init()
        tgt = brd.target
        dap = tgt.link
        
        dwt = tgt.cores[0].dwt
        itm = ITM(tgt.aps[0])
        tpiu = TPIU(tgt.aps[0])

#         print("has swo = {}".format(dap.has_swo()))
#         print("swo buffer = {} bytes".format(dap._swo_buffer_size))
        
        if not dap.has_swo():
            return

        itm.init()

        dap.swo_configure(False, SWO_CLK)
        dap.swo_configure(True, SWO_CLK)
#         print("configured swo for {} baud".format(SWO_CLK))
    
        itm.enable()
        tgt.write32(ITM.TERn, 0xffffffff)

        tpiu.init()
        tpiu.set_swo_clock(SWO_CLK, SYS_CLK)
#         print("setup itm and tpiu")
    
        v = (0 << DWT.DWT_CTRL_SYNCTAP_SHIFT) \
            | DWT.DWT_CTRL_CYCTAP_MASK \
            | (0 << DWT.DWT_CTRL_POSTINIT_SHIFT) \
            | (15 << DWT.DWT_CTRL_POSTRESET_SHIFT) \
            | DWT.DWT_CTRL_CYCCNTENA_MASK
        tgt.write32(DWT.DWT_CTRL, v)

        tgt.resume()
        
        dataFile = None
        logFile = None
        try:
            if WRITE_DATA:
                dataFile = open(SWO_DATA_FILENAME, 'wb')
            if WRITE_LOG:
                logFile = open(SWO_LOG_FILENAME, 'w')
            
            cyclesPerSecond = SYS_CLK # // 4
            
            # Create trace event graph.
            updateHandler = ScreenUpdateHandler(stdscr)
            logSink = SWOFileEventSink(logFile)
            sbSink = SamplbaerBufferedTimeEventSink(updateHandler)
            tee = pyOCD.trace.sink.TraceEventTee()
            parser = SWOParser(tgt)
            if WRITE_LOG:
                parser.connect(tee)
                tee.connect([logSink, sbSink])
            else:
                parser.connect(sbSink)

            # Clear cycle counter so it's more or less in sync with our run time.
            tgt.write32(DWT.DWT_CYCCNT, 0)

            dap.swo_control(True)

            t0 = time()
            t1 = 0.0
            lastStatus = -1
            
            while True:
                timestamp = (time() - t0) * 1000.0
                status, count, data = dap.swo_read()
#                 cyccnt = tgt.read32(DWT.DWT_CYCCNT) # TODO handle CYCCNT wrapping
                data = bytearray(data[:count])
                
                if status != lastStatus:
                    lastStatus = status
                    stdscr.addstr(0, 0, "SWO status: " + format_swo_status(status))
                    stdscr.refresh()

                if count:
                    parser.parse(data)

#                     if timestamp - t1 > 50.0:
#                         t1 = timestamp
#                         stdscr.refresh()
                        
        except KeyboardInterrupt:
            dap.swo_control(False)
        finally:
            if dataFile is not None:
                dataFile.close()
            if logFile is not None:
                logFile.flush()
                logFile.close()

    finally:
        if brd is not None:
            brd.uninit(False)

def main():
    logging.basicConfig(level=logging.WARNING)
    locale.setlocale(locale.LC_ALL, '')
    code = locale.getpreferredencoding()
    curses.wrapper(do_it)

if __name__ == "__main__":
    main()
