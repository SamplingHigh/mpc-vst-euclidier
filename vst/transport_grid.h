/* transport_grid.h -- host transport (audioMasterGetTime) -> the MIDI real-time stream the engine follows, anchored to
 * the SONG POSITION instead of to a pulse count.
 *
 * The engine's step is a pure function of its own `tick` (tick % ticks_per_step), and `tick` counts pulses from Start. If the
 * wrapper only fed it pulses, any lost pulse, a start in the middle of a 16th, or a loop wrap would shift every step for good.
 * So the wrapper also tells the engine WHERE it is: a Song Position Pointer (0xF2, in 16ths = 6 clocks) before the pulse at a
 * 16th boundary -- at Start, after every loop wrap / locate, and once per bar to heal a dropped pulse. The engine already
 * handles 0xF2 (tick = pos*6) while it follows an external clock. Pulses are still 24 per quarter, but they now always
 * arrive with tick == round(ppq*24).
 *
 * Header-only and engine-independent so grid_test.cpp can drive it with a model of the engine. Emitter E must provide:
 *   void start();            // 0xFA
 *   void stop();             // 0xFC
 *   void songpos(long s16);  // 0xF2, position in 16th notes (the emitter masks it to 14 bits)
 *   void pulse();            // 0xF8
 * See docs/MIDI_TIMING.md in mpc-vst-plugins for why. */
#pragma once
#include <algorithm>
#include <cmath>

struct TransportGrid {
    double sent_to = 0.0;     /* high-water mark: pulses before this song position have been sent */
    bool was_playing = false;
    bool need_sync = false;   /* next 16th boundary must be announced with a Song Position Pointer */
    bool need_start = false;  /* ... and preceded by Start (a fresh play, not a loop/locate) */

    /* Once per audio block. ppq = ppqPos at the start of this block. The pulses of THIS block, [ppq, ppq + blk), are sent now,
     * at its start, as StepRunner-8 sends its notes: they reach the engine (and its notes MPC) up to a block early instead of
     * a block late. The engine is a separate process and MPC's MIDI input comes after it, so earlier is closer to the grid;
     * sending the block that had just played left Euclidier ~3 ms plus that path behind the other tracks (a user had to set
     * a track delay on everything else to line up a TR-MPC driven by it).
     *
     * Looking ahead runs into the loop end: on a Force (MPC OS 3.9.1, 4-bar loop) the block before the wrap starts at 15.9990
     * and covers 16.0, the next reports 0.0052. MPC reports the loop as cycleStartPos/cycleEndPos (cycle flags set), so
     * loop_start/loop_end (loop_end <= loop_start: no loop) cut the block there: up to the loop end, then a Song Position
     * Pointer and the rest from the loop start. Without them a wrap is only seen a block later; its pulses then went out
     * already, under the pre-wrap position, and the re-announce comes at the next 16th. */
    template <class E>
    void block(E &e, bool playing, double ppq, double tempo, double sample_rate, int frames,
               double loop_start = 0.0, double loop_end = 0.0) {
        if (playing && !was_playing) { sent_to = ppq; need_sync = need_start = true; }
        else if (!playing && was_playing) { e.stop(); need_sync = need_start = false; }
        was_playing = playing;
        if (!playing) return;

        double blk = frames * (tempo > 0 ? tempo : 120.0) / (60.0 * (sample_rate > 0 ? sample_rate : 44100.0));
        double start = sent_to, end = ppq + blk;
        /* ppq behind what was sent by less than a beat: block jitter (MPC's ppq wobbles ~0.2 ms) or a tempo change (the host's
         * ppq steps back a little then crawls on); hold the high-water mark, so nothing plays twice. By more: a loop wrap the
         * loop range did not announce, or a locate. The block just before was covered by the look-ahead, so carry on from here,
         * less a little more than the jitter (a wrap reported at 0.0001 still plays the loop start's pulse at 0.0) -- except for
         * a wrap the host did not report: the look-ahead before it may already have sent the loop start's pulse (under the
         * pre-wrap position), so go from ppq itself rather than play it twice. */
        if (std::fabs(ppq - sent_to) > 1.0) {
            bool in_loop = loop_end > loop_start && ppq >= loop_start - 0.002 && ppq < loop_end;
            start = in_loop ? std::max(ppq - 0.002, loop_start) : ppq < sent_to ? ppq : ppq - 0.002;
            need_sync = true;
        }
        if (end <= start) return;

        if (loop_end > loop_start + 0.25 && ppq >= loop_start && ppq < loop_end && end > loop_end) {
            pulses(e, start, loop_end);
            need_sync = true;                          /* the loop start is announced before its first pulse */
            sent_to = loop_start + (end - loop_end);
            pulses(e, loop_start, sent_to);
        } else {
            pulses(e, start, end);
            sent_to = end;
        }
    }

private:
    template <class E>
    void pulses(E &e, double start, double end) {
        for (long m = (long)std::ceil(start * 24.0 - 1e-9); m / 24.0 < end - 1e-9; m++) {
            if (m < 0) continue;                       /* count-in / pre-roll */
            if (need_sync) {
                if (m % 6 != 0) continue;              /* wait for a 16th boundary: SPP can only name those */
                if (need_start) { e.start(); need_start = false; }
                e.songpos(m / 6);
                need_sync = false;
            } else if (m % 96 == 0) {
                e.songpos(m / 6);                      /* once per bar: heals a dropped or doubled pulse */
            }
            e.pulse();
        }
    }
};
