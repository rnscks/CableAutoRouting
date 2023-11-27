from DQNHyperparameters import DQNHyperparameters
from VoxelEnvironment import VoxelEnvironment
from ReplayMemory import ReplayMemory      
from DQN import DQN

class DQNAgent:
    def __init__(self, stateSize, actionSize, stepDone = 0):
        self.stateSize = stateSize
        self.actionSize = actionSize
        self.DQNpolicyNetwork = DQN(stateSize, actionSize).to(device = "cuda")
        self.DQNtargetNetwork = DQN(stateSize, actionSize).to(device = "cuda")
        
        self.DQNhyperParameter = DQNHyperparameters()
        self.DQNreplayMemory = ReplayMemory()
        self.StepDone = stepDone
        return
    
    def GetAction(self, state):
        eplisonEnd = self.DQNhyperParameter.EplisonEnd
        eplisonStart = self.DQNhyperParameter.EplisonStart
        eplisonDecay = self.DQNhyperParameter.EplisonDecay  
        sampleNumber - random.random()
        epsThreshold =  eplisonEnd + (eplisonStart - eplisonEnd) * np.exp(-1. * self.StepDone / eplisonDecay)
        self.StepDone += 1
        
        if (sampleNumber > epsThreshold):
            with torch.no_grad():
                return self.DQNpolicyNetwork(state).max(1)[1].view(1, 1)   
            
        return torch.tensor([[random.randrange(self.actionSize)]], dtype=torch.float32)
    
    
        
        
if (__name__ == "__main__"):
    ACTION_SIZE = 10
    environment = VoxelEnvironment()
    
    state = environment.Reset()
    dqnAgent = DQNAgent(stateSize = len(state), actionSize = ACTION_SIZE)
    
    
