function hN = DefineWeightedTerminalStates(statesString)
    global ACADO_;
    
    statesString = statesString(~isspace(statesString));
    states = strsplit(statesString, ';');
     
    ACADO_.yN = states;
    hN = evalin('base', ['[' statesString ']']);    
end