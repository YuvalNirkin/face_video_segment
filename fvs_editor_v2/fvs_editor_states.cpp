#include "fvs_editor_states.h"
#include "fvs_editor.h"

#include <iostream>//

// Boost
#include <boost/filesystem.hpp>

// Qt
#include <QTimer>
#include <QMessageBox>

using namespace boost::filesystem;

namespace fvs
{
    EditorSM::EditorSM(Editor * _editor) : editor(_editor)
    {
    }

    Inactive::Inactive(my_context ctx) : my_base(ctx), editor(nullptr)
    {
        editor = context<EditorSM>().editor;
    }

    void Inactive::onUpdate(const EvUpdate & event)
    {
    }

    sc::result Inactive::react(const EvStart &)
    {
        return transit< Active >();//
    }

    Active::Active(my_context ctx) : my_base(ctx), editor(nullptr)
    {
        editor = context<EditorSM>().editor;
        post_event(EvStart());
    }

    void Active::onSeek(const EvSeek & event)
    {
    }

    void Active::onStart(const EvStart & event)
    {
    }

    Paused::Paused(my_context ctx) : my_base(ctx), editor(nullptr)
    {
        editor = context<EditorSM>().editor;
    }

    void Paused::onUpdate(const EvUpdate& event)
    {
    }

    Playing::Playing(my_context ctx) : my_base(ctx), editor(nullptr)
    {
        editor = context<EditorSM>().editor;
    }

    Playing::~Playing()
    {
    }

    void Playing::onUpdate(const EvUpdate& event)
    {
    }

    void Playing::onTimerTick(const EvTimerTick& event)
    {
    }
}   // namespace sfl

