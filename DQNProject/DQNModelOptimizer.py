from DQNModel import DQNModel
from DQNHyperparameters import DQNHyperparameters
from ReplayMemory import ReplayMemory
from DQNAgent import DQNAgent
from torch import optimizer
import torch.nn as nn   

class DQNModelOptimizer:
    def __init__(self):
        pass
    def Run(self, DQNagent: DQNAgent,relayMemory: ReplayMemory, DQNhyperparmeter: DQNHyperparameter):
        if (DQNhyperparmeter.BatchSize > len(relayMemory)):
            return
        
        transitions = relayMemory.Sample(DQNhyperparmeter.BatchSize)    
        batch = Transtion(*zip(*transtions))
        
        nonFinalMask = torch.tensor(tuple(map(lambda s: s is not None, batch.nextState)), dtype = torch.bool)
        nonFinalNextStates = torch.cat([s for s in batch.nextState if s is not None])
        
        stateBatch = torch.cat(batch.state) 
        actionBatch = torch.cat(batch.action)
        rewardBatch = torch.cat(batch.reward)
        
        stateActionQValue = DQNagent.PolicyNetwork(stateBatch).gather(1, actionBatch)
        nextBatchQValue = torch.zeros(DQNhyperparmeter.BatchSize, dtype = torch.float32)
        
        with torch.no_grad():
            nextBatchQValue[nonFinalMask] = DQNagent.TargetNetwork(nonFinalNextStates).max(1)[0].detach()
            
        expectedQValue = (nextBatchQValue * DQNhyperparmeter.Gamma) + rewardBatch
        
        criterion = nn.SmoothL1Loss()
        loss = criterion(stateActionQValue, expectedQValue.unsqueeze(1))
        
        optimizer = optim.zeoro_grad()
        loss.backward()
        nn.utils.clip_grad_norm_(DQNagent.PolicyNetwork.parameters(), 100)

        optimizer.step()        
        
        
        return