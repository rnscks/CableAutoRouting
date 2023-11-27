class DQNHyperparameters:
    def __init__(self, learningRate = 1e-4, netWorkUpdatingRate = 0.005,discountFactor = 0.99, batchSize = 20, epsilonStart = 0.9, epsilonDecay = 1000, epsilonEnd = 0.05):
        self.NetWorkUpdatingRate = netWorkUpdatingRate  
        self.LearningRate = learningRate
        self.DiscountFactor = discountFactor
        self.BatchSize = batchSize
        self.EplisonStart = epsilonStart  
        self.EplisonDecay = epsilonDecay
        self.EplisonEnd = epsilonEnd
        return