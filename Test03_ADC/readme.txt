1,A0(ピンA0:ADC1 ch1)とA1(ピンA1:ADC1 ch2)に可変抵抗とかそういうのを接続する。
2,D0(ピンA12)にLEDを接続する。
3,電源を入れて可変抵抗を回すと、nucleo上のLED(ピンB3)とつないだLEDが角度によって点滅する。

確定した設定(上から順に)
Independent mode
DMA access mode enabled
1Cycle

Synchronous clock mode divided by 2
ADC 12-bit resolution
Right alignment
Enabled
Enabled
Disabled
Enabled
End of sequence of conversion
Overrun data preserved
Disabled

Enable 2
Regular Conversion launched by software
None
1(Channel1)
2(Channel2)

また、NVICメニューでDMA1 channel1 global interruptをチェック。
ADCのinterruptsにはチェックしない。
