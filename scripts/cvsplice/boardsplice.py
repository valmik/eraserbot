import numpy as np
import imutils
import cv2


class Stitcher():
    """docstring for Stitcher"""
    def __init__(self):
        self.isv3 = imutils.is_cv3()


    def stitch(self, full, new, ratio=0.75, reprojThresh=4.0, showMatches=False):
        # unpack the images, then detect keypoints and extract
        # local invariant descriptors from them

        imageA = np.zeros(full.shape, np.uint8)
        imageA[0:new.shape[0], 0:new.shape[1]] = new
        imageB = full

        maskA = np.zeros((full.shape[0], full.shape[1], 1), np.uint8)
        maskA[np.where((imageA!=[0,0,0]).all(axis=2))] = [255]
        m = cv2.moments(maskA, True)
        M = cv2.getRotationMatrix2D((m['m10']/m['m00'], m['m01']/m['m00']), 0, 1.5)
        maskA = cv2.warpAffine(maskA, M, (maskA.shape[1], maskA.shape[0]))

        maskB = np.zeros((full.shape[0], full.shape[1], 1), np.uint8)
        maskB[np.where((imageB!=[0,0,0]).all(axis=2))] = [255]
        m = cv2.moments(maskB, True)
        M = cv2.getRotationMatrix2D((m['m10']/m['m00'], m['m01']/m['m00']), 0, 1.5)
        maskB = cv2.warpAffine(maskB, M, (maskB.shape[1], maskB.shape[0]))

        combine = imageA + imageB
        combine = imutils.resize(combine, height=600)
        cv2.imshow("combine", combine)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        (kpsA, featuresA) = self.detectAndDescribe(imageA, maskB)
        (kpsB, featuresB) = self.detectAndDescribe(imageB, maskA)
 
        # match features between the two images
        M = self.matchKeypoints(kpsA, kpsB,
            featuresA, featuresB, ratio, reprojThresh)
 
        # if the match is None, then there aren't enough matched
        # keypoints to create a panorama
        if M is not None:

            # otherwise, apply a perspective warp to stitch the images
            # together
            (matches, H, status) = M

            result = cv2.warpPerspective(imageA, H,
                (imageA.shape[1], imageA.shape[0]))
        else:
            result = imageA

        # result[0:imageB.shape[0], 0:imageB.shape[1]] = imageB
        result[np.where((result==[0,0,0]).all(axis=2))] = imageB[np.where((result==[0,0,0]).all(axis=2))]

        # check to see if the keypoint matches should be visualized
        # if showMatches and M is not None:
        #     vis = self.drawMatches(imageA, imageB, kpsA, kpsB, matches,
        #         status)
 
        #     # return a tuple of the stitched image and the
        #     # visualization
        #     return (result, vis)
 
        # return the stitched image
        return result

    def detectAndDescribe(self, image, mask=None):
        # convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
 
        # check to see if we are using OpenCV 3.X
        if self.isv3:
            # detect and extract features from the image
            descriptor = cv2.xfeatures2d.SIFT_create()
            # descriptor.edgeThreshold = 2*descriptor.edgeThreshold
            (kps, features) = descriptor.detectAndCompute(image, mask)
            # print len(features)
            if not mask is None:
                masked = cv2.bitwise_and(image, image, mask = mask)
                masked = imutils.resize(masked, height=600)
                cv2.imshow("masked", masked)
                cv2.waitKey(0)
                cv2.destroyAllWindows()

 
        # otherwise, we are using OpenCV 2.4.X
        else:
            # detect keypoints in the image
            detector = cv2.FeatureDetector_create("SIFT")
            kps = detector.detect(gray)
 
            # extract features from the image
            extractor = cv2.DescriptorExtractor_create("SIFT")
            (kps, features) = extractor.compute(gray, kps)
 
        # convert the keypoints from KeyPoint objects to NumPy
        # arrays
        kps = np.float32([kp.pt for kp in kps])
 
        # return a tuple of keypoints and features
        return (kps, features)

    def matchKeypoints(self, kpsA, kpsB, featuresA, featuresB,
        ratio, reprojThresh):
        # compute the raw matches and initialize the list of actual
        # matches
        matcher = cv2.DescriptorMatcher_create("BruteForce")
        rawMatches = matcher.knnMatch(featuresA, featuresB, 2)
        matches = []
 
        # loop over the raw matches
        for m in rawMatches:
            # ensure the distance is within a certain ratio of each
            # other (i.e. Lowe's ratio test)
            if len(m) == 2 and m[0].distance < m[1].distance * ratio:
                matches.append((m[0].trainIdx, m[0].queryIdx))

        # computing a homography requires at least 4 matches
        if len(matches) > 10:
            # construct the two sets of points
            ptsA = np.float32([kpsA[i] for (_, i) in matches])
            ptsB = np.float32([kpsB[i] for (i, _) in matches])
 
            # compute the homography between the two sets of points
            # (H, status) = cv2.findHomography(ptsA, ptsB, cv2.RANSAC,
                # reprojThresh)
            # H[2,0] = 0
            # H[2,1] = 0

            H = cv2.estimateRigidTransform(ptsA, ptsB, False)
            status = []
            if not H is None:
                H = np.vstack((H, [0, 0, 1]))
            else: return None
            print H



            # return the matches along with the homograpy matrix
            # and status of each matched point
            return (matches, H, status)
 
        # otherwise, no homograpy could be computed
        return None

    def drawMatches(self, imageA, imageB, kpsA, kpsB, matches, status):
        # initialize the output visualization image
        (hA, wA) = imageA.shape[:2]
        (hB, wB) = imageB.shape[:2]
        vis = np.zeros((max(hA, hB), wA + wB, 3), dtype="uint8")
        vis[0:hA, 0:wA] = imageA
        vis[0:hB, wA:] = imageB
 
        # loop over the matches
        for ((trainIdx, queryIdx), s) in zip(matches, status):
            # only process the match if the keypoint was successfully
            # matched
            if s == 1:
                # draw the match
                ptA = (int(kpsA[queryIdx][0]), int(kpsA[queryIdx][1]))
                ptB = (int(kpsB[trainIdx][0]) + wA, int(kpsB[trainIdx][1]))
                cv2.line(vis, ptA, ptB, (0, 255, 0), 1)
 
        # return the visualization
        return vis
        





