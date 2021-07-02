#include "StateMachine.hpp"
namespace lib4253{

template<class State>
State StateMachine<State>::getState() const{
    return state;
}

template<typename State>
void StateMachine<State>::setState(State newState){
    if(state != newState){
        state = newState;
    }
}
}