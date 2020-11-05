#Ti电子设计竞赛 ——基于多悬翼飞行器设计一个绕障飞行器
#可以局部提取模块或者思路

#statement
others.py 用于openmv IDE，主要实现识别杆并进行亮灯和鸣声，属于前期模块代码，相对独立。
lsp.py       对雷达的数据的累计处理，由于代码中涉及到路径，因此需要根据自己的路径对应更改
ser.py      通信协议

#operation
openmv IDE :
openmvutils.py

Nano:
main.py
nanoutils.py
lsp.py
ser.py