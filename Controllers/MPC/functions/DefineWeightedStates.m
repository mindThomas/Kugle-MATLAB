function h = DefineWeightedStates(statesString)
    global ACADO_;
    
    statesString = statesString(~isspace(statesString));
    states = strsplit(statesString, ';');
     
    ACADO_.y = states;
    h = evalin('base', ['[' statesString ']']);    
end