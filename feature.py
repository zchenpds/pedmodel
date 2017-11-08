
from state import State

class Feature():
    def __init__(self, state, timestep):
        ''' construct the feature vector from a state object
        state: self state
        '''
        pos = state.pos
        for id, neighborState in state.neighborStates.items():
            neighborPos = neighborState.pos