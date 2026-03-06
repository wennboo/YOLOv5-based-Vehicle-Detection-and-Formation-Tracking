import torch.nn as nn
import torch.nn.init
import torch.nn.functional as F
from torchvision.models.optical_flow.raft import ResidualBlock


class ResBlock(nn.Module):
    def __init__(self, n_chans):
        super(ResBlock, self).__init__()
        self.conv = nn.Conv2d(n_chans, n_chans, kernel_size=3, padding=1, bias=False)
        self.batch_norm = nn.BatchNorm2d(num_features=n_chans)
        torch.nn.init.kaiming_normal_(self.conv.weight, nonlinearity='relu')
        torch.nn.init.constant_(self.batch_norm.weight, 0.5)
        torch.nn.init.zeros_(self.batch_norm.bias)

    def forward(self, x):
        out = self.conv(x)
        out = self.batch_norm(out)
        out = torch.relu(out)
        return out + x


class CBL(nn.Module):
    def __init__(self, c_in, c_out, size, stride, padding):
        super(CBL, self).__init__()
        self.conv = nn.Conv2d(c_in, c_out, size, stride, padding)
        self.batchnorm = nn.BatchNorm2d(c_out)
        # self.leakyrelu = nn.LeakyReLU()
        self.leakyrelu = nn.ReLU(inplace=True)

    def forward(self, x):
        out = self.conv(x)
        out = self.batchnorm(out)
        out = self.leakyrelu(out)
        return out


class CSP(nn.Module):
    def __init__(self, c_in):
        super(CSP, self).__init__()
        self.cbl = CBL(c_in, c_in, 3, 1, 1)
        self.res = ResBlock(c_in)
        self.conv = nn.Conv2d(c_in, int(c_in / 2), 3, 1, 1)
        self.batchnorm = nn.BatchNorm2d(c_in)

    def forward(self, x):
        b1 = self.cbl(x)
        b1 = self.res(b1)
        b1 = self.conv(b1)
        b2 = self.conv(x)
        b = torch.cat((b1, b2), dim=1)
        b = self.batchnorm(b)
        b = F.leaky_relu(b)
        b = self.cbl(b)
        return b


class Net(nn.Module):
    def __init__(self):
        super().__init__()
        # backbone
        self.cbl1 = CBL(3, 16, 2, 2, 0)
        self.csp1 = CSP(c_in=16)
        self.cbl2 = CBL(16, 24, 2, 2, 0)
        self.cbl3 = CBL(24, 32, 2, 2, 0)
        self.csp2 = CSP(c_in=32)
        self.cbl4 = CBL(32, 64, 2, 2, 0)

        self.cbl5 = CBL(64, 128, 2, 2, 0)
        self.csp3 = CSP(c_in=128)
        self.cbl6 = CBL(128, 64, 2, 2, 0)

        # neck
        self.upsample1 = nn.Upsample(scale_factor=4, mode='nearest')
        # self.concat1
        self.cbl7 = CBL(128, 32, 3, 1, 1)
        self.upsample2 = nn.Upsample(scale_factor=2, mode='nearest')
        # self.concat2

        # head
        self.cbl8 = CBL(64, 16, 4, 4, 0)
        self.cbl9 = CBL(16, 2, 4, 4, 0)
        self.fc1 = nn.Linear(50, 25)
        self.dropout = nn.Dropout(p=0.5)
        self.fc2 = nn.Linear(25, 10)
        self.fc3 = nn.Linear(10, 4)

    def forward(self, x):
        # backbone
        layer1 = self.csp1(self.cbl1(x))
        layer2 = self.cbl2(layer1)
        layer3 = self.csp2(self.cbl3(layer2))
        layer4 = self.cbl4(layer3)
        layer5 = self.csp3(self.cbl5(layer4))

        # neck
        layer6 = self.cbl6(layer5)
        layer7 = self.upsample1(layer6)
        layer8 = torch.cat((layer4, layer7), 1)  # N*C*H*W
        layer9 = self.cbl7(layer8)
        layer10 = self.upsample2(layer9)
        layer11 = torch.cat((layer3, layer10), 1)

        # head
        layer12 = self.cbl8(layer11)
        layer13 = self.cbl9(layer12)
        layer13_ = layer13.view(layer13.shape[0], -1)
        # A dropout layer with rate = .5 after the first
        # connected layer prevents co-adaptation between layers
        layer14 = self.fc1(layer13_)
        layer14 = torch.sigmoid(layer14)
        layer15 = self.fc2(layer14)
        layer15 = torch.sigmoid(layer15)
        layer16 = self.fc3(layer15)
        layer16 = torch.sigmoid(layer16)
        return layer16
