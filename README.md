# BeaconLoc

定位算法雏形，使用蓝牙信标进行定位。


用例：

```shell
python src/main.py <beacon_coord_file> <beacon_rssi_file>
```

其中，`beacon_coord_file` 是信标坐标文件，`beacon_rssi_file` 是手机接收信号强度文件。

核心启动逻辑在`utils/common.py`的`process_beacon_data_for_pos`函数，该函数接受信标坐标和信号强度数据，返回手机位置。具体会示例化一个`BeaconLocalization`对象，然后根据文件传输的内容调用`update`函数。


以HTTP形式进行数据传输，demo见`src/server.py`。
