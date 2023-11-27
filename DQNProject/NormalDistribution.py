import numpy as np
import matplotlib.pyplot as plt


class Colors:
    def __init__(self) -> None:
        self.ColorContainer = ['red', 'blue', 'green', 'yellow', 'black', 'orange', 'purple', 'pink', 'brown', 'gray']
        self.ColorIndex = 0
        return
    
    def GetColor(self):
        color = self.ColorContainer[self.ColorIndex]
        self.ColorIndex = (self.ColorIndex + 1) % len(self.ColorContainer)
        return color
    


class NormalDistribution:
    def __init__(self) -> None:
        plt.rcParams['font.weight'] = 'bold'  # Set font weight to bold
        plt.rcParams['font.size'] = 20  # Increase font size
        return  
    
    def AppendPlot(self, mean, sigma, color = 'green'):
        # Generate random data from a normal distribution
        data = np.random.normal(mean, sigma, 1000)
        plt.hist(data, bins=30, density=True, alpha=0.7, color=color)

        # Plot the probability density function (PDF) of the normal distribution
        x = np.linspace(mean - 3*sigma, mean + 3*sigma, 100)
        pdf = np.exp(-0.5 * ((x - mean) / sigma)**2) / (sigma * np.sqrt(2*np.pi))
        plt.plot(x, pdf, color=color, linewidth=2)
        return 
    
    def Plot(self):
        # Set plot title and labels
        plt.title('Distribution of Rewards', fontweight='bold', fontsize=25)
        plt.xlabel('Value of Rewards', fontweight='bold', fontsize=20)
        plt.ylabel('Normalized Probability Density', fontweight='bold', fontsize=20)

        # Show the plot
        plt.show()
        return

if (__name__ == '__main__'):    
    normalDistribution = NormalDistribution()
    colors = Colors()   
    # Generate random data from a normal distribution
    meanVector = [10, -4, 2, -8, 3, 5, 6, -3, 0, 9]  # mean
    sigma = 1  # standard deviation

    for mean in meanVector:
        normalDistribution.AppendPlot(mean, sigma, colors.GetColor())  
        
normalDistribution.Plot()
