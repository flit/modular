/*
 * Copyright (c) 2017-2018 Immo Software
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its contributors may
 *   be used to endorse or promote products derived from this software without
 *   specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#if !defined(_CARD_MANAGER_H_)
#define _CARD_MANAGER_H_

#include "singleton.h"
#include "argon/argon.h"
#include "stack_sizes.h"
#include <stdint.h>

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

/*!
 * @brief Manages the SD card and card insertion/removal detection.
 */
class CardManager : public Singleton<CardManager>
{
public:
    CardManager();
    ~CardManager()=default;

    void init();

    bool is_card_present() { return _isCardPresent; }

    //! @brief Tell the card manager that an error occurred while accessing the card.
    //!
    //! The card manager will immediately check presence. If the card is missing, then
    //! it will send a #kCardRemoved event to the UI.
    void report_card_error();

    //! @brief Execute commands to determine whether there is a card present.
    bool check_presence();

protected:
    Ar::ThreadWithStack<kCardThreadStack> _thread;
    Ar::RunLoop _runloop;
    Ar::TimerWithMemberCallback<CardManager> _cardDetectTimer;
    bool _isCardPresent;    //!< Debounced card presence.
    bool _isCardInited;     //!< Whether SD_CardInit() was successful.
    bool _debounceCardDetect;   //!< Indicates whether we are currently debouncing card detect.
    bool _isImmediateCheckPending;  //!< Used to coalesce check_card_remove() performs on the runloop.

    bool get_card_status();

    void card_thread();

    void check_card_removed();
    static void check_card_removed_stub(void * param);

    void handle_card_detect_timer(Ar::Timer * timer);
};

} // namespace slab

#endif // _CARD_MANAGER_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
