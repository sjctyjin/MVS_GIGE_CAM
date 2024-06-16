import sys
sys.path.append("./MvImport")
from MvCameraControl_class import *
import numpy as np
import cv2


def main():
    # 創建相機對象
    cam = MvCamera()

    # 設備信息列表
    deviceList = MV_CC_DEVICE_INFO_LIST()
    tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE

    # 枚舉設備
    ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
    if ret != 0:
        print("Enum devices failed! ret[0x%x]" % ret)
        return

    if deviceList.nDeviceNum == 0:
        print("No device found!")
        return

    # 選擇第一台相機並創建句柄
    ret = cam.MV_CC_CreateHandle(deviceList.pDeviceInfo[0])
    if ret != 0:
        print("Create handle failed! ret[0x%x]" % ret)
        return

    # 打開設備
    ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive)
    if ret != 0:
        print("Open device failed! ret[0x%x]" % ret)
        return

    # 設置取流緩衝區大小
    ret = cam.MV_CC_SetIntValue("GevSCPD", 1000)
    if ret != 0:
        print("Set GevSCPD failed! ret[0x%x]" % ret)

    # 設置為連續取流模式
    ret = cam.MV_CC_SetEnumValue("AcquisitionMode", MV_ACQ_MODE_CONTINUOUS)
    if ret != 0:
        print("Set Acquisition Mode failed! ret[0x%x]" % ret)
        return

    # 開始取流
    ret = cam.MV_CC_StartGrabbing()
    if ret != 0:
        print("Start grabbing failed! ret[0x%x]" % ret)
        return

    try:
        while True:
            # 獲取一幀影像
            data = MV_FRAME_OUT_INFO_EX()
            stOutFrame = MV_FRAME_OUT()
            ret = cam.MV_CC_GetOneFrameTimeout(byref(stOutFrame), sizeof(stOutFrame), 1000)
            if ret == 0:
                # 轉換成numpy array
                frame_data = (c_ubyte * stOutFrame.stFrameInfo.nFrameLen)()
                memmove(frame_data, stOutFrame.pBufAddr, stOutFrame.stFrameInfo.nFrameLen)
                img_array = np.array(frame_data).reshape(stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth,
                                                         -1)
                # 顯示影像
                cv2.imshow("Image", img_array)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                print("Get one frame failed! ret[0x%x]" % ret)
    finally:
        # 停止取流
        cam.MV_CC_StopGrabbing()
        # 關閉設備
        cam.MV_CC_CloseDevice()
        # 銷毀句柄
        cam.MV_CC_DestroyHandle()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
