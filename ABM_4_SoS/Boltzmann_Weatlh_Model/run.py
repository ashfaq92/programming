import sys
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt

from money_model import MoneyModel

model = MoneyModel(100, 10, 10)

for _ in range(20):
    model.step()

# get the data
data = model.datacollector.get_agent_vars_dataframe()
# assign histogram colors
palette = {"Green": "green", "Blue": "blue", "Mixed": "purple"}
g = sns.histplot(data=data, x="Wealth", hue="Ethnicity", discrete=True, palette=palette)
g.set(title="Wealth distribution", xlabel="Wealth", ylabel="number of agents")


plt.show()
