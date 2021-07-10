#pragma once
namespace lib4253{

template <class State>
class StateMachine{
    public:
    StateMachine() = default;
    ~StateMachine() = default;

    State getState() const{
        return state;
    }

    void setState(State newState){
        state = newState;
    }

    private:
    State state;  
};
}
