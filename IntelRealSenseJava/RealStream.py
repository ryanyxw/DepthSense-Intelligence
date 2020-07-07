# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

"""
OpenCV and Numpy Point cloud Software Renderer

This sample is mostly for demonstration and educational purposes.
It really doesn't offer the quality or performance that can be
achieved with hardware acceleration.

Usage:
------
Mouse: 
    Drag with left button to rotate around pivot (thick small axes), 
    with right button to translate and the wheel to zoom.

Keyboard: 
    [p]     Pause
    [r]     Reset View
    [d]     Cycle through decimation values
    [z]     Toggle point scaling
    [c]     Toggle color source
    [s]     Save PNG (./out.png)
    [e]     Export points to ply (./out.ply)
    [q\ESC] Quit
"""

import math
import time
import cv2
import numpy as np
import pyrealsense2 as rs

class AppState:

    def __init__(self, *args, **kwargs):
        self.WIN_NAME = 'RealSense'
        self.pitch, self.yaw = math.radians(-10), math.radians(-15)
        self.translation = np.array([0, 0, -1], dtype=np.float32)
        self.distance = 2
        self.prev_mouse = 0, 0
        self.mouse_btns = [False, False, False]
        self.paused = False
        
        #This is to control how clear the image is
        self.decimate = 2
        self.scale = True
        self.color = True

    def reset(self):
        self.pitch, self.yaw, self.distance = 0, 0, 2
        self.translation[:] = 0, 0, -1

    @property
    def rotation(self):
        Rx, _ = cv2.Rodrigues((self.pitch, 0, 0))
        Ry, _ = cv2.Rodrigues((0, self.yaw, 0))
        return np.dot(Ry, Rx).astype(np.float32)

    @property
    def pivot(self):
        return self.translation + np.array((0, 0, self.distance), dtype=np.float32)


state = AppState()

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30) #DO NOT CHANGE CONFIG
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#(Type, width, height, format, framerate)

# Start streaming
pipeline.start(config) #Config represents the finished configuration of the item

# Get stream profile and camera intrinsics
#Gets basic data from the camera
profile = pipeline.get_active_profile()
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
depth_intrinsics = depth_profile.get_intrinsics()
w, h = depth_intrinsics.width, depth_intrinsics.height



# Processing blocks
pc = rs.pointcloud()
#Decimation decreases the sample rate of a signal by removing samples from the data stream
decimate = rs.decimation_filter()
decimate.set_option(rs.option.filter_magnitude, 2 ** state.decimate)
colorizer = rs.colorizer()


#Controls all the dragging around
def mouse_cb(event, x, y, flags, param):

    if event == cv2.EVENT_LBUTTONDOWN:
        state.mouse_btns[0] = True

    if event == cv2.EVENT_LBUTTONUP:
        state.mouse_btns[0] = False

    if event == cv2.EVENT_RBUTTONDOWN:
        state.mouse_btns[1] = True

    if event == cv2.EVENT_RBUTTONUP:
        state.mouse_btns[1] = False

    if event == cv2.EVENT_MBUTTONDOWN:
        state.mouse_btns[2] = True

    if event == cv2.EVENT_MBUTTONUP:
        state.mouse_btns[2] = False

    if event == cv2.EVENT_MOUSEMOVE:

        h, w = out.shape[:2]
        dx, dy = x - state.prev_mouse[0], y - state.prev_mouse[1]

        if state.mouse_btns[0]:
            state.yaw += float(dx) / w * 2
            state.pitch -= float(dy) / h * 2

        elif state.mouse_btns[1]:
            dp = np.array((dx / w, dy / h, 0), dtype=np.float32)
            state.translation -= np.dot(state.rotation, dp)

        elif state.mouse_btns[2]:
            dz = math.sqrt(dx**2 + dy**2) * math.copysign(0.01, -dy)
            state.translation[2] += dz
            state.distance -= dz

    if event == cv2.EVENT_MOUSEWHEEL:
        dz = math.copysign(0.1, flags)
        state.translation[2] += dz
        state.distance -= dz

    state.prev_mouse = (x, y)


cv2.namedWindow(state.WIN_NAME, cv2.WINDOW_AUTOSIZE)
cv2.resizeWindow(state.WIN_NAME, w, h)
cv2.setMouseCallback(state.WIN_NAME, mouse_cb)


def project(v):
    """project 3d vector array to 2d"""
    h, w = out.shape[:2]
    view_aspect = float(h)/w

    # ignore divide by zero for invalid depth (errstate)
    with np.errstate(divide='ignore', invalid='ignore'):
        proj = v[:, :-1] / v[:, -1, np.newaxis] * \
            (w*view_aspect, h) + (w/2.0, h/2.0)

    # near clipping
    znear = 0.03
    proj[v[:, 2] < znear] = np.nan
    return proj


def view(v):
    """apply view transformation on vector array"""
    return np.dot(v - state.pivot, state.rotation) + state.pivot - state.translation

#pt1 and pt2 must be in the form of view(Deprojection)
def line3d(out, pt1, pt2, color=(0x80, 0x80, 0x80), thickness=1):
    """draw a 3d line from pt1 to pt2"""
    p0 = project(pt1.reshape(-1, 3))[0]
    p1 = project(pt2.reshape(-1, 3))[0]
    if np.isnan(p0).any() or np.isnan(p1).any():
        return
    p0 = tuple(p0.astype(int))
    p1 = tuple(p1.astype(int))
    rect = (0, 0, out.shape[1], out.shape[0])
    inside, p0, p1 = cv2.clipLine(rect, p0, p1)
    if inside:
        cv2.line(out, p0, p1, color, thickness, cv2.LINE_AA)


#Draws the grid on the point cloud
def grid(out, pos, rotation=np.eye(3), size=1, n=10, color=(0x80, 0x80, 0x80)):
    """draw a grid on xz plane"""
    pos = np.array(pos)
    s = size / float(n)
    s2 = 0.5 * size
    
    #Looping through the rows and columns
    for i in range(0, n+1):
        x = -s2 + i*s
        line3d(out, view(pos + np.dot((x, 0, -s2), rotation)),
               view(pos + np.dot((x, 0, s2), rotation)), color)
    for i in range(0, n+1):
        z = -s2 + i*s
        line3d(out, view(pos + np.dot((-s2, 0, z), rotation)),
               view(pos + np.dot((s2, 0, z), rotation)), color)


#Draws the axis on the pointcloud
def axes(out, pos, rotation=np.eye(3), size=0.075, thickness=2):
    """draw 3d axes"""
    line3d(out, pos, pos +
           np.dot((0, 0, size), rotation), (0xff, 0, 0), thickness)
    line3d(out, pos, pos +
           np.dot((0, size, 0), rotation), (0, 0xff, 0), thickness)
    line3d(out, pos, pos +
           np.dot((size, 0, 0), rotation), (0, 0, 0xff), thickness)


#Frustum is the area where the camera can pick the objects up(detection)
#Draws the guidlines that appear around the camera's field of view
def frustum(out, intrinsics, color=(0x40, 0x40, 0x40)):
    """draw camera's frustum"""
    orig = view([0, 0, 0])
    w, h = intrinsics.width, intrinsics.height

    #d is distance in meters from the center
    for d in range(1, 6, 2):
        def get_point(x, y):
            p = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], d)
            line3d(out, orig, view(p), color)
            return p

        top_left = get_point(0, 0)
        top_right = get_point(w, 0)
        bottom_right = get_point(w, h)
        bottom_left = get_point(0, h)

        line3d(out, view(top_left), view(top_right), color)
        line3d(out, view(top_right), view(bottom_right), color)
        line3d(out, view(bottom_right), view(bottom_left), color)
        line3d(out, view(bottom_left), view(top_left), color)
#        print(bottom_left)
#        print(view(bottom_left))


def pointcloud(out, verts, texcoords, color, painter=True):
    """draw point cloud with optional painter's algorithm"""
    if painter:
        # Painter's algo, sort points from back to front

        # get reverse sorted indices by z (in view-space)
        # https://gist.github.com/stevenvo/e3dad127598842459b68
        v = view(verts)
        s = v[:, 2].argsort()[::-1]
        proj = project(v[s])
    else:
        proj = project(view(verts))

    if state.scale:
        proj *= 0.5**state.decimate

    h, w = out.shape[:2]

    # proj now contains 2d image coordinates
    j, i = proj.astype(np.uint32).T

    # create a mask to ignore out-of-bound indices
    im = (i >= 0) & (i < h)
    jm = (j >= 0) & (j < w)
    m = im & jm
#    m = True

    cw, ch = color.shape[:2][::-1]
    if painter:
        # sort texcoord with same indices as above
        # texcoords are [0..1] and relative to top-left pixel corner,
        # multiply by size and add 0.5 to center
        v, u = (texcoords[s] * (cw, ch) + 0.5).astype(np.uint32).T
    else:
        v, u = (texcoords * (cw, ch) + 0.5).astype(np.uint32).T
    # clip texcoords to image
    np.clip(u, 0, ch-1, out=u)
    np.clip(v, 0, cw-1, out=v)

    # perform uv-mapping
    out[i[m], j[m]] = color[u[m], v[m]]

#================================================================================================================================================================









class FoundObject(object):
    def __init__(self, row, col):
        self.bigRow = row
        self.bigCol = col
        self.smallRow = row
        self.smallCol = col
    #This adds a point to the found object
    def addPoint(self, newRow, newCol):
        if self.bigRow < newRow:
            self.bigRow = newRow
        elif self.smallRow > newRow:
            self.smallRow = newRow
        if self.bigCol < newCol:
            self.bigCol = newCol
        elif self.smallCol > newCol:
            self.smallCol = newCol
    #THis determines if a column is between, on the left, or on the right of an object
    def isBetween(self, newCol):
        if newCol > self.smallCol:
            if newCol < self.bigCol:
                return True
            else:
                return "right"
        else:
            return "left"
    def __repr__(self):
        return "Row between " + repr(self.smallRow) + " and " + repr(self.bigRow) + ", col between " + repr(self.smallCol) + ", " + repr(self.bigCol)
            
#You dont have to search top-left directions
searchingPattern = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

'''
def getAllObject(gameState, depth_intrinsics, maxDiff):
    objectList = []
#    for row in range(depth_intrinsics.height):
#        for col in range(depth_intrinsics.row):
    for row in range(depth_intrinsics[0]):
        for col in range(depth_intrinsics[1]):
            if gameState[row][col] != 0:
                currentObject = FoundObject(row, col)
                gameState, currentObject = getAllSurrounding(gameState, currentObject, row, col, maxDiff, searchingPattern, depth_intrinsics)
                objectList += [currentObject]
    return objectList
'''

#This method returns the object's biggest col num if the targetcol is found to be within an object, else it returns -1
def binarySearchObject(objectList, targetCol):
    if len(objectList) == 0:
        return -1
    left = 0
    right = len(objectList)-1
    mid = (left+right)//2
#    print("left = " + repr(left))
#    print("right = " + repr(right))
#    print("mid = " + repr(mid))
#    print("objectList = " + repr(objectList))
#    print("targetCol = " + repr(targetCol))
    while right - left > 1:
#        print("left = " + repr(left))
#        print("right = " + repr(right))
#        print("mid = " + repr(mid))
        '''
        currentCol = objectList[mid].smallCol
        if currentCol == targetCol:
            return objectList[mid].bigCol
        elif currentCol < targetCol:
            left = mid
            mid = (left+right)//2
        elif currentCol > targetCol:
            right = mid
            mid = (left+right)//2
        else:
            print("ERRRORRRRRR")
        '''
        compare = objectList[mid].isBetween(targetCol)
        if compare == True:
            return objectList[mid].bigCol
        elif compare == "left":
            right = mid
            mid = (left+right)//2
        elif compare == "right":
            left = mid
            mid = (left+right)//2
        else:
            print("ERRRORRRRR")
    '''
    if objectList[left].smallCol == targetCol:
        print("lies within object of " + repr(objectList[left]))
        return objectList[left].bigCol
    elif objectList[right].smallCol == targetCol:
        print("lies within object of " + repr(objectList[right]))
        return objectList[right].bigCol
    '''
    if objectList[left].isBetween(targetCol) == True:
#        print("object = " + repr(objectList[left]))
#        print("targetCol = " + repr(targetCol))
#        print("isBetween = " + repr(objectList[left].isBetween(targetCol)))
#        print("lies within object of " + repr(objectList[left]))
        return objectList[left].bigCol
    elif objectList[right].isBetween(targetCol) == True:
#        print("lies within object of " + repr(objectList[right]))
        return objectList[right].bigCol
    return -1

#This method inserts another object into an objectList in order (binary search and insert)
def binaryInsertObject(objectList, currentObject):
    if len(objectList) == 0:
        objectList += [currentObject]
        return objectList
    left = 0
    right = len(objectList) - 1
    mid = (left + right) // 2
    while left - right > 1:
        
        if left == right:
            if objectList[left].smallCol > currentObject.smallCol:
                objectList.insert(left, currentObject)
            else:
                objectList.insert(left+1, currentObject)
        if left > right:
            objectList.insert(left, currentObject)
        
        currentCol = objectList[mid].smallCol
        if currentCol < currentObject.smallCol:
            left = mid + 1
            mid = (left + right) // 2
        elif currentCol > currentObject.smallCol:
            right = mid - 1
            mid = (left + right) // 2
        else:
            print("INSERT ERRORRR")
            
    if objectList[left].smallCol < currentObject.smallCol:
        if objectList[right].smallCol < currentObject.smallCol:
            objectList.insert(right + 1, currentObject)
        else:
            objectList.insert(right, currentObject)
    else:
        objectList.insert(left, currentObject)
    '''
    if objectList[left].smallCol > currentObject.smallCol:
        objectList.insert(left, currentObject)
    elif objectList[left].smallCol < currentObject.smallCol:
        objectList.insert(left + 1, currentObject)
    '''
    return objectList


#This method sets every point within the smallCol to bigCol to become zero
def setColRangeToZero(gameState, smallCol, bigCol):
    length = len(gameState)
    for row in range(length):
        for col in range(smallCol, bigCol + 1):
            gameState[row][col] = 0;
    return gameState

#This method gets all the objects in a given game state
def getAllObject(gameState, depth_intrinsics, maxDiff):
    objectList = []
#    for row in range(depth_intrinsics.height):
#        for col in range(depth_intrinsics.row):
    row = 0
    col = 0
    while row < depth_intrinsics.height:
        while col < depth_intrinsics.width:
#            print("----------------------------------------------------------------")
#            print("We are checking row: " + repr(row) + " and col: " + repr(col))
            if gameState[row][col] != 0:
#                print("Point value is " + repr(gameState[row][col]))
                currentObject = FoundObject(row, col)
#                print("gameState before is \n" + repr(gameState))
                gameState, currentObject = getAllSurrounding(gameState, currentObject, row, col, maxDiff, searchingPattern, depth_intrinsics, 0)
                #In case there is a glitch in the detection, where there is only one point 
#                print("gameState after is \n" + repr(gameState))
#                print("currentObject Found is " + repr(currentObject))
                col += 1
                if currentObject.bigCol - currentObject.smallCol >=2:
#                    print("objectList before is " + repr(objectList))
                    objectList = binaryInsertObject(objectList, currentObject)
#                    print("objectList after is " + repr(objectList))
                    #This sets everything within these columns to become zero
#                    print("gameState changed from \n" + repr(gameState))
                    gameState = setColRangeToZero(gameState, currentObject.smallCol, currentObject.bigCol)
#                    print("to \n" + repr(gameState))
#                else:
#                    print("object ignored because its too small")
            else:
#                print("point has a value of zero")
                tempCol = binarySearchObject(objectList, col)
                
                if tempCol == -1:
 #                   print("lies in no object")
                    col += 1
                    continue
                else:
#                    print("cols jumpting from " + repr(col) + " to " + repr(tempCol))
                    col = tempCol
                    col += 1
#        print("NEXT ROW ===============================================================")
        col = 0
        row += 1
    return objectList


#This gets all the surrounding pixels that have a close depth value to the target pixel
#Uses recursion, and it always deletes the currentRow and currentCol, dosn't delete the next ones
def getAllSurrounding(gameState, currentObject, currentRow, currentCol, maxDiff, searchingPattern, depth_intrinsics, count):
    if count > 80:
        return
    currentValue = gameState[currentRow][currentCol]
    gameState[currentRow][currentCol] = 0
#    print("BASEEEE = " + repr(currentRow) + ", " + repr(currentCol))
    for currentDirection in searchingPattern:
        addRow, addCol = currentDirection
        newRow = currentRow + addRow
        newCol = currentCol + addCol
#        print("now searching + " + repr(newRow) + " and " + repr(newCol))
#        if newCol < 0 or newCol >= depth_intrinsics.width or newRow < 0 or newRow >= depth_intrinsics.height:
        if newCol < 0 or newCol >= depth_intrinsics.width or newRow < 0 or newRow >= depth_intrinsics.height:
#            print("skipped because out of bounds on row of " + repr(newRow) + ", and col of " + repr(newCol))
            continue
        newValue = gameState[newRow][newCol]
        if newValue == 0:
            
            continue
        
        if abs(newValue - currentValue) <= maxDiff:
#            print("MATCH")
            currentObject.addPoint(newRow, newCol)
            getAllSurrounding(gameState, currentObject, newRow, newCol, maxDiff, searchingPattern, depth_intrinsics, count+1)
    return gameState, currentObject

def testGetAllSurrounding():
    '''
    gameState = [[0, 0, 0, 0, 4, 6, 5, 6, 8, 14, 13, 0, 0], 
                 [0, 0, 0, 3, 3, 5, 3, 3, 5, 11, 16, 10, 0], 
                 [0, 0, 0, 0, 2, 4, 5, 5, 4, 13, 14, 13, 0],
                 [0, 0, 0, 5, 4, 3, 5, 2, 4, 14, 13, 15, 0]]
#    currentObject = FoundObject(0, 4)
    maxDiff = 2
    depth_intrinsics = (4, 13)
#    print("GameState before = \n" + repr(gameState))
#    getAllSurrounding(gameState, currentObject, 0, 4, maxDiff, searchingPattern, depth_intrinsics)
#    print("GameState after = \n" + repr(gameState))
#    print(currentObject)
    print(gameState)
    objectList = getAllObject(gameState, depth_intrinsics, maxDiff)
    print(gameState)
    print(objectList)
    
    
    
    gameState = [[2, 0, 4, 0, 6, 0, 8, 0, 10], 
                 [0, 12, 0, 14, 0, 16, 0, 18, 0], 
                 [20, 0, 22, 0, 24, 0, 26, 0, 28],
                 [0, 30, 0, 32, 0, 34, 0, 36, 0]]
#    currentObject = FoundObject(0, 4)
    maxDiff = 2
    depth_intrinsics = (4, 9)
#    print("GameState before = \n" + repr(gameState))
#    getAllSurrounding(gameState, currentObject, 0, 4, maxDiff, searchingPattern, depth_intrinsics)
#    print("GameState after = \n" + repr(gameState))
#    print(currentObject)
    print(gameState)
    objectList = getAllObject(gameState, depth_intrinsics, maxDiff)
    print(gameState)
    print(objectList)
    '''
    gameState = [[0.0, 0.0, 0.54, 0.83, 0.64, 1.1, 1.23, 1.21, 0.0, 0.0, 0.0], 
                 [0.0, 0.0, 0.23, 0.38, 0.45, 0.98, 1.07, 1.14, 1.21, 0.0, 0.0], 
                 [0.0, 0.32, 0.31, 0.42, 0.0, 0.0, 1.1, 1.21, 1.32, 1.23, 0.0],
                 [0.0, 0.0, 0.35, 0.37, 0.52, 0.0, 0.0, 1.09, 1.27, 1.25, 1.22]]
#    currentObject = FoundObject(0, 4)
    maxDiff = 0.2
    depth_intrinsics = (4, 11)
#    print("GameState before = \n" + repr(gameState))
#    getAllSurrounding(gameState, currentObject, 0, 4, maxDiff, searchingPattern, depth_intrinsics)
#    print("GameState after = \n" + repr(gameState))
#    print(currentObject)
    print(gameState)
    objectList = getAllObject(gameState, depth_intrinsics, maxDiff)
    print(gameState)
    print(objectList)
    
    

#testGetAllSurrounding()



#This method finds the longest streak of empty spaces and returns the degree to which someone should turn (0 to 1)
def findLongestStreak(objectList):
    longestStreak = -1
    streakPosition = -1
    totalObject = len(objectList)
    for objectIndex in range(1, totalObject):
        currentDif = objectList[objectIndex].smallCol - objectList[objectIndex - 1].bigCol
        if currentDif > longestStreak:
            longestStreak = currentDif
            streakPosition = (objectList[objectIndex].smallCol + objectList[objectIndex - 1].bigCol)//2
    return streakPosition / 160

def translateToWords(moveDecimal):
    if moveDecimal <= 0.5:
        return "right"
    else:
        return "left"



#================================================================================================================================================================







out = np.empty((h, w, 3), dtype=np.uint8)

#For the one-time prints in the while true functions
countVariable = 0
#This is used as the searching pattern for getting nearby pixels (row, col)
searchingPattern = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

while True:
    # Grab camera data
    if not state.paused:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        #This is to delete occcasional pixels in order to make process more smooth
        depth_frame = decimate.process(depth_frame)

        # Grab new intrinsics (may be changed by decimation)
        #GETS THE INTRINSIC
        depth_intrinsics = rs.video_stream_profile(
            depth_frame.profile).get_intrinsics()
        w, h = depth_intrinsics.width, depth_intrinsics.height
#        print(w, h)
        
        #Depth image and depth frame are essentially the same thing (one is array)
#        depth_image = np.asanyarray(depth_frame.get_data())
#        print(depth_image)
        
        #This creates an array that stores all the depth information
        depthData = depth_frame.as_depth_frame()
        depthArray = [([0] * w) for row in range(h)]
        for width in range(w):
            for height in range(h):
                depthArray[height][width] = depthData.get_distance(width, height)
        
        
        #This gets the actual RGB values for pixels on the array
        color_image = np.asanyarray(color_frame.get_data())

        #This gets the abstract colors assigned to depth
        depth_colormap = np.asanyarray(
            colorizer.colorize(depth_frame).get_data())
        
        #If it is in color mode, use the RGB value, else use abstract color
        if state.color:
            mapped_frame, color_source = color_frame, color_image
        else:
            mapped_frame, color_source = depth_frame, depth_colormap


        #Any changes to make to illustration must change depth_frame
        points = pc.calculate(depth_frame)
        pc.map_to(mapped_frame)

        # Pointcloud data to arrays
        v, t = points.get_vertices(), points.get_texture_coordinates()
        verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
        texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv
        
        
        
        
        #Testing Area
#        countVariable += 1
        if (countVariable % 50 == 0):
#            print("Height = " + repr(len(depthArray)))
#            print("Width = " + repr(len(depthArray[0])))
            
            '''
            best = -1
            for row in range(len(depthArray)):
                for col in range(len(depthArray[0])):
                    if depthArray[row][col] > best:
                        best = depthArray[row][col]
            print(best)
            print(depthArray[0])
            oneTimeBool = False
            '''
            maxDiff = 1
            objectList = getAllObject(depthArray, depth_intrinsics, maxDiff)
            moveDecimal = findLongestStreak(objectList)
            print("====================================================")
            print("objectList = " + repr(objectList))
            print("moveDecimal = " + repr(moveDecimal))
            print(translateToWords(moveDecimal))
            
            
    countVariable += 1
    # Render
    now = time.time()

    out.fill(0)
    
    #Draws all the grids and lines
    grid(out, (0, 0.5, 1), size=1, n=10)
    frustum(out, depth_intrinsics)
    axes(out, view([0, 0, 0]), state.rotation, size=0.1, thickness=1)


    #CRUCIALLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
    #CRUCIALLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
    #CRUCIALLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
    #CRUCIALLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
    
    #deproject_pixel_to_point: final parameter is like polar coordinates
    #How you draw a line (must be after out.fill(0))
    p1 = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [0, 0], 2)
    p2 = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [319, 239], 2)
    p3 = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [319, 239], 3)
    #Always use deproject_pixel_to_point to get the 3D coordinates
    
    line3d(out, view(p1), view(p2))
    line3d(out, view(p1), view(p3))
    
    #Personal Test:
    

    #Always draw something in out
#    line3d(out, view([w/3, h/2, 0]), view([w*2/3, h/2, 0]))
    if not state.scale or out.shape[:2] == (h, w):
        pointcloud(out, verts, texcoords, color_source)
    else:
        tmp = np.zeros((h, w, 3), dtype=np.uint8)
        pointcloud(tmp, verts, texcoords, color_source)
        tmp = cv2.resize(
            tmp, out.shape[:2][::-1], interpolation=cv2.INTER_NEAREST)
        np.putmask(out, tmp > 0, tmp)

    if any(state.mouse_btns):
        axes(out, view(state.pivot), state.rotation, thickness=4)

    dt = time.time() - now

    cv2.setWindowTitle(
        state.WIN_NAME, "RealSense (%dx%d) %dFPS (%.2fms) %s" %
        (w, h, 1.0/dt, dt*1000, "PAUSED" if state.paused else ""))

    cv2.imshow(state.WIN_NAME, out)
    key = cv2.waitKey(1)

    if key == ord("r"):
        state.reset()
        oneTimeBool = True

    if key == ord("p"):
        state.paused ^= True
        oneTimeBool = True

    if key == ord("d"):
        
        #This sets up the clarity of the output values to be 3 levels (mod3)
        state.decimate = (state.decimate + 1) % 3
        decimate.set_option(rs.option.filter_magnitude, 2 ** state.decimate)
        oneTimeBool = True

    if key == ord("z"):
        state.scale ^= True
        oneTimeBool = True

    if key == ord("c"):
        state.color ^= True
        oneTimeBool = True

    if key == ord("s"):
        cv2.imwrite('./out.png', out)
        oneTimeBool = True

    if key == ord("e"):
        points.export_to_ply('./out.ply', mapped_frame)
        oneTimeBool = True

    if key in (27, ord("q")) or cv2.getWindowProperty(state.WIN_NAME, cv2.WND_PROP_AUTOSIZE) < 0:
        break

# Stop streaming
pipeline.stop()
