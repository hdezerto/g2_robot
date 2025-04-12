#!/usr/bin/env python


import numpy as np
import cv2 as cv
import glob
import os
import matplotlib.pyplot as plt



# Most likely single time use
def calibrateCamera(showPics = True):
    print('hek')
    # Read Image
    root = os.getcwd()
    print(root)
    calibrationDir = os.path.join(root, 'calibration')
    imgPathList = glob.glob(os.path.join(calibrationDir, '*.jpg'))
    #print(imgPathList)
    # Initialize 
    nRows = 6 # tbd
    nCols = 6 # tbd
    termCriteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER,30, 0.001)
    worldPtsCur = np.zeros((nRows*nCols,3), np.float32)
    worldPtsCur[:,:2] = np.mgrid[0:nRows,0:nCols].T.reshape(-1,2)
    worldPtsList = []
    imgPtsList = []

    # Find corners
    for curImgPath in imgPathList:
        print(curImgPath)
        imgBGR = cv.imread(curImgPath)
        
        imgGray = cv.cvtColor(imgBGR, cv.COLOR_BGR2GRAY)
        #imgGray = cv.equalizeHist(imgGray)
        cv.imshow('image',imgGray)
        cv.waitKey(1000)
        cornersFound, cornersOrg = cv.findChessboardCorners(imgGray,(nRows,nCols), None)
        print(cornersFound)

        if cornersFound == True:
            worldPtsList.append(worldPtsCur)
            cornersRefined = cv.cornerSubPix(imgGray, cornersOrg, (11,11), (-1,-1), termCriteria)
            imgPtsList.append(cornersRefined)
            if showPics:
                cv.drawChessboardCorners(imgBGR, (nRows,nCols), cornersRefined, cornersFound)
                cv.imshow('Chessboard', imgBGR)
                cv.waitKey(500)
        elif cornersFound == False:
            continue
        cv.destroyAllWindows()

        # Calibrate 
        repError, camMatrix, distCoeff, rvecs, tvecs = cv.calibrateCamera(
            worldPtsList, imgPtsList, imgGray.shape[::-1], None, None)
        print('Camera Matrix: \n', camMatrix)
        print('Reproj Error (pixels): {:.4f}'.format(repError))
        
        # Save Calibration Parameters 
        curFolder = os.path.dirname(os.path.abspath(__file__))
        paramPath = os.path.join(curFolder, 'calibration.npz')
        np.savez(paramPath, 
                 repError=repError,
                 camMatrix=camMatrix,
                 distCoeff = distCoeff,
                 rvecs=rvecs,
                 tvecs=tvecs)
        return camMatrix, distCoeff
    
    def removeDistorion(camMatrix, distCoeff):
        root = os.getcwd()
        imgPath = os.path.join(root, 'demoImages//distortion2.jpg')
        img = cv.imread(imgPath)
        height, width = img.shape[:2]
        camMatrixNew,roi, = cv.getOptimalNewCameraMatrix(camMatrix,distCoeff,(width,height),1, (width,height))
        imgUndist = cv.undistort(img,camMatrix, distCoeff, None, camMatrixNew)

        # Draw Kube to See Distortion Change
        cv.line(img, (1769,103),(1780,922),(255,255,255),2)
        cv.line(imgUndist, (1769,103),(1780,922),(255,255,255),2)

        plt.figure()
        plt.subplot(121)
        plt.imshow(img)
        plt.subplot(122)
        plt.imshow(imgUndist)
        plt.show()

def runCalibration():
    calibrateCamera(showPics=True)

def runRemoveDistortion():
    camMatrix, distCoeff = calibrateCamera(showPics=False)
    runRemoveDistortion(camMatrix,distCoeff)

def createImages():
    root = os.getcwd()
    calibrationDir = os.path.join(root, 'calibration')
    cap = cv.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        exit()
    image_count = 0
    while True:
        # Read a frame from the webcam
        ret, frame = cap.read()

        if not ret:
            print("Error: Failed to capture image.")
            break

        cv.imshow('Webcam Feed', frame)

        key = cv.waitKey(1) & 0xFF
        if key == ord('q'):  # Press 'q' to exit
            break
        elif key == ord('s'):  # Press 's' to save the current frame
            image_filename = os.path.join(calibrationDir, f"image_{image_count}.jpg")
            cv.imwrite(image_filename, frame)
            print(f"Saved: {image_filename}")
            image_count += 1

        

def main():
    runCalibration()
    #createImages()

if __name__== '__main__':
    #runRemoveDistortion()
    main()