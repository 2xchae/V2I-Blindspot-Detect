import torch
import torch.nn as nn

class CNN(nn.Module):
    
    def __init__(self):
        super().__init__()

        #레이어 정의
        self.layer1 = nn.Conv1d(2, 32, 5, 1, padding='same')
        self.layer2 = nn.Conv1d(32, 64, 5, 1, padding='same')
        self.relu = nn.ReLU()
        self.pooling = nn.AdaptiveAvgPool1d(1)
        self.flatten = nn.Flatten()
        self.dropout = nn.Dropout(p=0.3)
        self.linear = nn.Linear(64,3)
        

    def forward(self, x): # x.shape는 (batch, 2, 40)
        x1 = self.layer1(x) # (batch, 32, 40)
        x1 = self.relu(x1) # (batch, 32, 40)
        x2 = self.layer2(x1) # (batch, 64, 40)
        x2 = self.relu(x2) # (batch, 64, 40)
        features = self.flatten(self.pooling(x2)) # (batch, 64)
        features = self.dropout(features)
        features = self.linear(features)

        return features