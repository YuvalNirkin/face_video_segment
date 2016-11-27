#ifndef __FVS_EDITOR_STATES_H__
#define __FVS_EDITOR_STATES_H__

// Boost
#include <boost/statechart/event.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/in_state_reaction.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/mpl/list.hpp>

// Qt
#include <QtWidgets/QWidget>


namespace sc = boost::statechart;
namespace mpl = boost::mpl;

namespace fvs
{
    // Forward declarations
    struct Inactive;
    struct Active;
    struct Paused;
    struct Playing;
    class Editor;

    // State Machine 
    struct EditorSM : sc::state_machine< EditorSM, Inactive >
    {
        EditorSM(Editor* _editor);
        Editor* editor;
    };

    // Events

    struct EvPlayPause : sc::event< EvPlayPause > {};
    struct EvUpdate : sc::event< EvUpdate > {};
    struct EvStart : sc::event< EvStart > {};
    struct EvReset : sc::event< EvReset > {};
    struct EvTimerTick : sc::event< EvTimerTick > {};
    struct EvSeek : sc::event< EvSeek > 
    {
        EvSeek(int _i) : i(_i) {}
        int i;
    };

    // States

    struct Inactive : sc::state<Inactive, EditorSM>
    {
        Inactive(my_context ctx);

        void onUpdate(const EvUpdate& event);
        sc::result react(const EvStart&);

        typedef mpl::list<
            sc::in_state_reaction<EvUpdate, Inactive, (void(Inactive::*)(const EvUpdate&))(&Inactive::onUpdate)>,
            sc::custom_reaction< EvStart > > reactions;

        Editor* editor;
    };

    struct Active : sc::state<Active, EditorSM, Paused>
    {
        Active(my_context ctx);

        void onSeek(const EvSeek& event);
        void onStart(const EvStart& event);

        typedef mpl::list<
            sc::in_state_reaction<EvSeek, Active, (void(Active::*)(const EvSeek&))(&Active::onSeek)>,
            sc::in_state_reaction<EvStart, Active, (void(Active::*)(const EvStart&))(&Active::onStart)>,
            sc::transition< EvReset, Inactive > > reactions;

        Editor* editor;
    };

    struct Paused : sc::state<Paused, Active>
    {
        Paused(my_context ctx);

        void onUpdate(const EvUpdate& event);

        typedef mpl::list<
            sc::in_state_reaction<EvUpdate, Paused, (void(Paused::*)(const EvUpdate&))(&Paused::onUpdate)>,
            sc::transition< EvPlayPause, Playing > > reactions;

        Editor* editor;
    };

    // States
    struct Playing : sc::state<Playing, Active>
    {
        Playing(my_context ctx);
        ~Playing();

        void onUpdate(const EvUpdate& event);
        void onTimerTick(const EvTimerTick& event);

        typedef mpl::list<
            sc::in_state_reaction<EvUpdate, Playing, (void(Playing::*)(const EvUpdate&))(&Playing::onUpdate)>,
            sc::in_state_reaction<EvTimerTick, Playing, (void(Playing::*)(const EvTimerTick&))(&Playing::onTimerTick)>,
            sc::transition< EvPlayPause, Paused > > reactions;

        Editor* editor;
        int timer_id = 0;
    };
}   // namespace fvs





#endif // __FVS_EDITOR_STATES_H__