import sys
from ctypes import *

sys.path.append("../MvImport")
from MvCameraControl_class import *
import numpy as np
import cv2

def input_num_camera(deviceList):
    # nConnectionNum = input("please input the number of the device to connect:")
    nConnectionNum = 0
    if int(nConnectionNum) >= deviceList.nDeviceNum:
        print("intput error!")
        sys.exit()
    return nConnectionNum
def main():
    # 創建相機對象
    cam = MvCamera()

    # 枚舉相機
    tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE
    deviceList = MV_CC_DEVICE_INFO_LIST()
    # 枚举设备
    ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
    print(ret)
    if ret != 0:
        print("enum devices fail! ret[0x%x]" % ret)
        sys.exit()
    if deviceList.nDeviceNum == 0:
        print("find no device!")
        sys.exit()
    print("Find %d devices!" % deviceList.nDeviceNum)
    nConnectionNum = input_num_camera(deviceList)
    stDeviceList = cast(deviceList.pDeviceInfo[int(nConnectionNum)], POINTER(MV_CC_DEVICE_INFO)).contents
    # 選擇相機[0]並創建句柄
    ret = cam.MV_CC_CreateHandle(stDeviceList)

    if ret != 0:
        print("Create handle failed! ret[0x%x]" % ret)
        sys.exit()

    # 打開設備
    ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive)
    if ret != 0:
        print("Open device failed! ret[0x%x]" % ret)
        sys.exit()

    # 設置為連續取流模式
    ret = cam.MV_CC_SetEnumValue("AcquisitionMode", MV_ACQ_MODE_CONTINUOUS)
    if ret != 0:
        print("Set Acquisition Mode failed! ret[0x%x]" % ret)
        sys.exit()

    # 開始取流
    ret = cam.MV_CC_StartGrabbing()
    if ret != 0:
        print("Start grabbing failed! ret[0x%x]" % ret)
        sys.exit()

    try:
        # stOutFrame = MV_FRAME_OUT()
        # stFrameInfo = MV_FRAME_OUT_INFO_EX()
        stOutFrame = MV_FRAME_OUT()
        memset(byref(stOutFrame), 0, sizeof(stOutFrame))
        while True:
            ret = cam.MV_CC_GetImageBuffer(stOutFrame, 1000)
            if ret == 0:
                print("Get one frame: Width[%d], Height[%d], nFrameNum[%d]" %
                      (stOutFrame.stFrameInfo.nWidth, stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nFrameNum))

                # 將圖像數據轉換成NumPy數組
                frame_data = (c_ubyte * stOutFrame.stFrameInfo.nFrameLen)()
                memmove(frame_data, stOutFrame.pBufAddr, stOutFrame.stFrameInfo.nFrameLen)
                img_array = np.array(frame_data).reshape(stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth,
                                                         -1)

                # 影像格式轉換
                if stOutFrame.stFrameInfo.enPixelType == PixelType_Gvsp_Mono8:
                    img_array = cv2.cvtColor(img_array, cv2.COLOR_GRAY2BGR)
                elif stOutFrame.stFrameInfo.enPixelType == PixelType_Gvsp_BGR8_Packed:
                    img_array = img_array.reshape((stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth, 3))

                cv2.imshow('Camera Stream', img_array)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                print("No frame is grabbed. Error code:", ret)
    finally:
        # 停止取流並關閉設備
        cam.MV_CC_StopGrabbing()
        cam.MV_CC_CloseDevice()
        cam.MV_CC_DestroyHandle()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
