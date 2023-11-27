import matplotlib.pyplot as plt 

class DQNLearningPlotter:
    def __init__(self) -> None:
        self.EpisodeDurations = []
        self.AvgRewards = []
        pass
    def Plot(self) -> None:
        plt.title('Average Duration per Episode')
        plt.xlabel('Episode')
        plt.ylabel('Duration')
        plt.plot(self.EpisodeDurations)
        plt.show()
        return
    
    def Push(self, episodeDuration, avgReward):
        self.EpisodeDurations.append(episodeDuration)
        self.AvgRewards.append(avgReward)
        return
    