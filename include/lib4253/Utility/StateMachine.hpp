#pragma once
namespace lib4253{

template <class State>
class StateMachine{
    public:
        StateMachine() = default;
        ~StateMachine() = default;

        State state;
        State getState() const;
        void setState(State newState);
};
}