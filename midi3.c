/*
 * Layer-3 of MIDI support
 *
 * (C) Christoph van Wullen, DL1YCF
 *
 *
 * In most cases, a certain action only makes sense for a specific
 * type. For example, changing the VFO frequency will only be implemeted
 * for MIDI_WHEEL, and TUNE off/on only with MIDI_KNOB.
 *
 * However, changing the volume makes sense both with MIDI_KNOB and MIDI_WHEEL.
 */
#include <gtk/gtk.h>

#include "actions.h"
#include "midi.h"

void DoTheMidi(int action, enum ACTIONtype type, int val) {

    //g_print("%s: action=%d type=%d val=%d\n",__FUNCTION__,action,type,val);

    switch(type) {
      case MIDI_KEY:
        schedule_action(action, val?PRESSED:RELEASED, 0);
        break;
      case MIDI_KNOB:
        schedule_action(action, ABSOLUTE, val);
        break;
      case MIDI_WHEEL:
        schedule_action(action, RELATIVE, val);
        break;
      default:
        // other types cannot happen for MIDI
        break;
    }
}
