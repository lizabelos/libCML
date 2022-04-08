#include "cml/features/corner/ORB.h"
#if CML_HAVE_OPENCV
#include "cml/features/corner/OpenCV.h"
#endif

#include "cml/image/Filter.h"
#include "ORBPattern.cpp"

const int PATCH_SIZE = 31;
const int HALF_PATCH_SIZE = 15;
const int EDGE_THRESHOLD = 19;

CML::Features::ORB::ORB(Ptr<AbstractFunction, Nullable> parent) :
AbstractFunction(parent)
{
    mFilter = Filter::gaussian1d(7, 2);

    reinitialize();


}

void CML::Features::ORB::reinitialize() {

    mvScaleFactor.resize(mNLevels.i());
    mvLevelSigma2.resize(mNLevels.i());
    mvScaleFactor[0]=1.0f;
    mvLevelSigma2[0]=1.0f;
    for(int i=1; i<mNLevels.i(); i++)
    {
        mvScaleFactor[i]=mvScaleFactor[i-1]*mScaleFactor.f();
        mvLevelSigma2[i]=mvScaleFactor[i]*mvScaleFactor[i];
    }

    mvInvScaleFactor.resize(mNLevels.i());
    mvInvLevelSigma2.resize(mNLevels.i());
    for(int i=0; i<mNLevels.i(); i++)
    {
        mvInvScaleFactor[i]=1.0f/mvScaleFactor[i];
        mvInvLevelSigma2[i]=1.0f/mvLevelSigma2[i];
    }

    mnFeaturesPerLevel.resize(mNLevels.i());
    float factor = 1.0f / mScaleFactor.f();
    float nDesiredFeaturesPerScale = mNumCorner.i()*(1 - factor)/(1 - (float)pow((double)factor, (double)mNLevels.i()));

    int sumFeatures = 0;
    for( int level = 0; level < mNLevels.i()-1; level++ )
    {
        mnFeaturesPerLevel[level] = fastRound(nDesiredFeaturesPerScale);
        sumFeatures += mnFeaturesPerLevel[level];
        nDesiredFeaturesPerScale *= factor;
    }
    mnFeaturesPerLevel[mNLevels.i()-1] = std::max(mNumCorner.i() - sumFeatures, 0);

    //const int npoints = 512;
    //const Point* pattern0 = (const Point*)bit_pattern_31_;
    //std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));

    //This is for orientation
    // pre-compute the end of a row in a circular patch
    umax.resize(HALF_PATCH_SIZE + 1);

    int v, v0, vmax = floor(HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1);
    int vmin = ceil(HALF_PATCH_SIZE * sqrt(2.f) / 2);
    const double hp2 = HALF_PATCH_SIZE*HALF_PATCH_SIZE;
    for (v = 0; v <= vmax; ++v)
        umax[v] = fastRound(sqrt(hp2 - v * v));

    // Make sure we are symmetric
    for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v)
    {
        while (umax[v0] == umax[v0 + 1])
            ++v0;
        umax[v] = v0;
        ++v0;
    }


    mImages.resize(mNLevels.i());
    mBluredImages.resize(mNLevels.i());
    mBluredImagesTmpA.resize(mNLevels.i());
    mBluredImagesTmpB.resize(mNLevels.i());
    mBluredImagesTmpC.resize(mNLevels.i());
    mBluredImagesTmpD.resize(mNLevels.i());
    mBluredImagesTmpE.resize(mNLevels.i());

    mAllKeypoints.resize(mNLevels.i());
    for (int i = 0; i < mNLevels.i(); i++) {
        mAllKeypoints[i].reserve(mNumCorner.i());
    }

    mCorners.reserve(mNumCorner.i());
    mDescriptors.reserve(mNumCorner.i());
}

void CML::Features::ORB::compute(const CaptureImage &captureImage) {

    #pragma omp single
    {
        mCorners.clear();
        mDescriptors.clear();
    }

    #pragma omp single
    if (mUseCache.b() && captureImage.getPath() != "") {
        std::string orbCachePath = captureImage.getPath() + ".orb" + std::to_string(mNumCorner.i());
        FILE *f = fopen(orbCachePath.c_str(), "rb");
        if (f != nullptr) {
            logger.important("Using cached ORB");
            int count = 0;
            fread(&count, sizeof(int), 1, f);
            mCorners.resize(count);
            mDescriptors.resize(count);
            for (int i = 0; i < count; i++) {
                fread(&mCorners[i], sizeof(Corner), 1, f);
            }
            for (int i = 0; i < count; i++) {
                fread(mDescriptors[i].data(), mDescriptors[i].size(), 1, f);
            }
            fclose(f);
        }
    }

    if (!mCorners.empty()) {
        return;
    }

    assertDeterministic("Original image", captureImage.getGrayImage(0).eigenMatrix().sum());

    captureImage.getGrayImage(0).castToUChar(mImages[0]);
    assertDeterministic("Image 0", mImages[0].eigenMatrix().sum());
    for (int i = 1; i < mNLevels.i(); i++) {
        int imgWidth = (float)captureImage.getWidth(0) * mvInvScaleFactor[i];
        imgWidth = imgWidth / 16;
        imgWidth = imgWidth * 16;
        mImages[i - 1].resize(imgWidth, captureImage.getHeight(0) * imgWidth / captureImage.getWidth(0), mImages[i]);
        assertDeterministic("Image", mImages[i].eigenMatrix().sum());
    }

    computeKeyPointsOctTree();

    for (int level = 0; level < mNLevels.i(); ++level)
    {
        int nkeypointsLevel = (int)mAllKeypoints[level].size();

        if(nkeypointsLevel == 0) {
            continue;
        }

        // preprocess the resized image
        if (mBlur.b()) {
            mImages[level].cast<float>(mBluredImagesTmpA[level]);
            mBluredImagesTmpA[level].convolution1D(mFilter, mBluredImagesTmpB[level]);
            mBluredImagesTmpB[level].transpose(mBluredImagesTmpC[level]);
            mBluredImagesTmpC[level].convolution1D(mFilter, mBluredImagesTmpD[level]);
            mBluredImagesTmpD[level].transpose(mBluredImagesTmpE[level]);
            mBluredImagesTmpE[level].castToUChar<unsigned char>(mBluredImages[level]);

            //mImages[level].cast<float>().convolution(mFilter, mBluredImages[level]);
        } else {
            mBluredImages[level].copyToThis(mImages[level]);
        }

        // Compute the descriptors
        computeDescriptors(mBluredImages[level], mAllKeypoints[level], desc);

        // Scale keypoint coordinates
        #pragma omp single
        {
            if (level != 0) {
                float scale = mvScaleFactor[level]; //getScale(level, firstLevel, scaleFactor);
                for (Corner &keypoint: mAllKeypoints[level]) {
                    keypoint.scalePoint(scale);
                }
            }
            // And add the keypoints to the output
            for (int j = 0; j < mAllKeypoints[level].size(); j++) {
                mAllKeypoints[level][j].padPoint(0.5, 0.5);
                if (captureImage.getGrayImage(0).goodPosition(mAllKeypoints[level][j].x(), mAllKeypoints[level][j].y())) {
                    mCorners.emplace_back(mAllKeypoints[level][j]);
                    mDescriptors.emplace_back(desc[j]);
                }
            }
        }

    }


    #pragma omp single
    if (mUseCache.b() && captureImage.getPath() != "") {
        std::string orbCachePath = captureImage.getPath() + ".orb" + std::to_string(mNumCorner.i());
        FILE *f = fopen(orbCachePath.c_str(), "wb");
        if (f != nullptr) {
            logger.important("Using cached ORB");
            int count = mCorners.size();
            fwrite(&count, sizeof(int), 1, f);
            for (int i = 0; i < count; i++) {
                fwrite(&mCorners[i], sizeof(Corner), 1, f);
            }
            for (int i = 0; i < count; i++) {
                fwrite(mDescriptors[i].data(), mDescriptors[i].size(), 1, f);
            }
            fclose(f);
        }
    }

    //logger.important("Extracted " + std::to_string(mCorners.size()) + " orb coners");

}

void CML::Features::ORB::computeKeyPointsOctTree() {

    #pragma omp single
    {

#if CML_USE_OPENMP
        int ompNumThread = omp_get_num_threads();
#else
        int ompNumThread = 1;
#endif
        mFast = new FAST[ompNumThread];

        for (int i = 0; i < mNLevels.i(); i++) {
            mAllKeypoints[i].clear();
        }
    }

    const float W = 30;

    for (int level = 0; level < mNLevels.i(); level++)
    {
        const int minBorderX = EDGE_THRESHOLD-3;
        const int minBorderY = minBorderX;
        const int maxBorderX = mImages[level].getWidth() - EDGE_THRESHOLD+3;
        const int maxBorderY = mImages[level].getHeight() - EDGE_THRESHOLD+3;

#pragma omp single
        {
            vToDistributeKeys.clear();
            vToDistributeKeys.reserve(mNumCorner.i() * 10);
        }

        const float width = (maxBorderX-minBorderX);
        const float height = (maxBorderY-minBorderY);

        const int nCols = width/W;
        const int nRows = height/W;
        const int wCell = ceil(width/nCols);
        const int hCell = ceil(height/nRows);

        for(int i=0; i<nRows; i++)
        {
            const float iniY =minBorderY+i*hCell;
            float maxY = iniY+hCell+6;

            if(iniY>=maxBorderY-3) {
                continue;
            }
            if(maxY>maxBorderY) {
                maxY = maxBorderY;
            }

            #pragma omp for ordered
            for(int j=0; j<nCols; j++)
            {
                const float iniX =minBorderX+j*wCell;
                float maxX = iniX+wCell+6;
                if(iniX>=maxBorderX-6) {
                    continue;
                }
                if(maxX>maxBorderX) {
                    maxX = maxBorderX;
                }

#if CML_USE_OPENMP
                int tid = omp_get_thread_num();
#else
                int tid = 0;
#endif

                List<Corner> vKeysCell;
                mFast[tid].compute(mImages[level].crop(iniX, iniY, maxX - iniX, maxY - iniY), vKeysCell, mIniThFAST.i());

                assertDeterministic("FAST (try 1)", vKeysCell.size());

                if(vKeysCell.empty() && mIncreaseThreshold.b())
                {
                    mFast[tid].compute(mImages[level].crop(iniX, iniY, maxX - iniX, maxY - iniY), vKeysCell, mMinThFAST.i());
                    assertDeterministic("FAST (try 2)", vKeysCell.size());
                }

                #pragma omp ordered
                if(!vKeysCell.empty())
                {
                    for(Corner &corner : vKeysCell)
                    {
                        Corner copy = corner;
                        copy.padPoint(j * wCell, i * hCell);
                        vToDistributeKeys.push_back(copy);
                    }
                }

            }
        }

        #pragma omp single
        {
            List<Corner> &keypoints = mAllKeypoints[level];
            distributeOctTree(vToDistributeKeys, keypoints, minBorderX, maxBorderX, minBorderY, maxBorderY,
                              mnFeaturesPerLevel[level], level);
            assertDeterministic("Final number of keypoints", keypoints.size());


            const int scaledPatchSize = PATCH_SIZE * mvScaleFactor[level];

            // Add border to coordinates and scale information
            const int nkps = keypoints.size();
            for (int i = 0; i < nkps; i++) {
                keypoints[i].padPoint(minBorderX, minBorderY);
                keypoints[i].setLevel(level);
                keypoints[i].setScaleFactor(mScaleFactor.f());
                keypoints[i].setSize(scaledPatchSize);
            }
            logger.debug("ORB computed " + std::to_string(keypoints.size()) + " at level " + std::to_string(level));
        }

    }

    // compute orientations
    #pragma omp for
    for (int level = 0; level < mNLevels.i(); ++level) {
        computeOrientation(mImages[level], mAllKeypoints[level], umax);
    }
}

void CML::Features::ORB::distributeOctTree(const List<Corner>& vToDistributeKeys, List<Corner>& vResultKeys, const int &minX, const int &maxX, const int &minY, const int &maxY, const int &N, const int &level) {

    // Compute how many initial nodes
    const int nIni = fastRound(static_cast<float>(maxX-minX)/(maxY-minY));

    const float hX = static_cast<float>(maxX-minX)/nIni;

    LinkedList<ExtractorNode> lNodes;

    List<ExtractorNode*> vpIniNodes;
    vpIniNodes.resize(nIni);

    for(int i=0; i<nIni; i++)
    {
        ExtractorNode ni;
        ni.UL = Vector2i(hX*static_cast<float>(i),0);
        ni.UR = Vector2i(hX*static_cast<float>(i+1),0);
        ni.BL = Vector2i(ni.UL.x(),maxY-minY);
        ni.BR = Vector2i(ni.UR.x(),maxY-minY);
        ni.vKeys.reserve(vToDistributeKeys.size());

        lNodes.push_back(ni);
        vpIniNodes[i] = &lNodes.back();
    }

    //Associate points to childs
    for(size_t i=0;i<vToDistributeKeys.size();i++)
    {
        const Corner &kp = vToDistributeKeys[i];
        vpIniNodes[kp.x() / hX]->vKeys.push_back(kp);
    }


    LinkedList<ExtractorNode>::iterator lit = lNodes.begin();

    while(lit!=lNodes.end())
    {
        if(lit->vKeys.size()==1)
        {
            lit->bNoMore=true;
            lit++;
        }
        else if(lit->vKeys.empty())
            lit = lNodes.erase(lit);
        else
            lit++;
    }



    bool bFinish = false;

    int iteration = 0;

    List<Pair<int,ExtractorNode*>> vSizeAndPointerToNode;
    vSizeAndPointerToNode.reserve(lNodes.size()*4);

    while(!bFinish)
    {
        iteration++;

        int prevSize = lNodes.size();

        lit = lNodes.begin();

        int nToExpand = 0;

        vSizeAndPointerToNode.clear();

        while(lit!=lNodes.end())
        {
            if(lit->bNoMore)
            {
                // If node only contains one point do not subdivide and continue
                lit++;
                continue;
            }
            else
            {
                // If more than one point, subdivide
                ExtractorNode n1,n2,n3,n4;
                lit->divideNode(n1,n2,n3,n4);

                // Add childs if they contain points
                if(n1.vKeys.size()>0)
                {
                    lNodes.push_front(n1);
                    if(n1.vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(Pair<int,ExtractorNode*>(n1.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n2.vKeys.size()>0)
                {
                    lNodes.push_front(n2);
                    if(n2.vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(Pair<int,ExtractorNode*>(n2.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n3.vKeys.size()>0)
                {
                    lNodes.push_front(n3);
                    if(n3.vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(Pair<int,ExtractorNode*>(n3.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n4.vKeys.size()>0)
                {
                    lNodes.push_front(n4);
                    if(n4.vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(Pair<int,ExtractorNode*>(n4.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }

                lit=lNodes.erase(lit);

                continue;
            }
        }

        // Finish if there are more nodes than required features
        // or all nodes contain just one point
        if((int)lNodes.size()>=N || (int)lNodes.size()==prevSize)
        {
            bFinish = true;
        }
        else if(((int)lNodes.size()+nToExpand*3)>N)
        {

            while(!bFinish)
            {

                prevSize = lNodes.size();

                List<Pair<int,ExtractorNode*>> vPrevSizeAndPointerToNode = vSizeAndPointerToNode;
                vSizeAndPointerToNode.clear();

                sort(vPrevSizeAndPointerToNode.begin(),vPrevSizeAndPointerToNode.end(), [](const Pair<int,ExtractorNode*> & a, const Pair<int,ExtractorNode*> & b) -> bool
                {
                    if (a.first != b.first) {
                        return a.first < b.first;
                    }
                    return a.second->hash() < b.second->hash();
                });
                //assertThrow(vPrevSizeAndPointerToNode[0].first <= vPrevSizeAndPointerToNode[vPrevSizeAndPointerToNode.size()-1].first, "Whut ?");
                for(int j=vPrevSizeAndPointerToNode.size()-1;j>=0;j--)
                {
                    ExtractorNode n1,n2,n3,n4;
                    vPrevSizeAndPointerToNode[j].second->divideNode(n1,n2,n3,n4);

                    // Add childs if they contain points
                    if(n1.vKeys.size()>0)
                    {
                        lNodes.push_front(n1);
                        if(n1.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(Pair<int,ExtractorNode*>(n1.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n2.vKeys.size()>0)
                    {
                        lNodes.push_front(n2);
                        if(n2.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(Pair<int,ExtractorNode*>(n2.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n3.vKeys.size()>0)
                    {
                        lNodes.push_front(n3);
                        if(n3.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(Pair<int,ExtractorNode*>(n3.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n4.vKeys.size()>0)
                    {
                        lNodes.push_front(n4);
                        if(n4.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(Pair<int,ExtractorNode*>(n4.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }

                    lNodes.erase(vPrevSizeAndPointerToNode[j].second->lit);


                    if((int)lNodes.size()>=N)
                        break;
                }

                if((int)lNodes.size()>=N || (int)lNodes.size()==prevSize)
                    bFinish = true;

            }
        }
    }

    // Retain the best point in each node
    for(LinkedList<ExtractorNode>::iterator lit=lNodes.begin(); lit!=lNodes.end(); lit++)
    {
        List<Corner> &vNodeKeys = lit->vKeys;
        Corner *pKP = &vNodeKeys[0];
        float maxResponse = pKP->response();

        for(size_t k=1;k<vNodeKeys.size();k++)
        {
            int currentResponse = vNodeKeys[k].response();
            if(currentResponse > maxResponse)
            {
                pKP = &vNodeKeys[k];
                maxResponse = currentResponse;
            }
        }

        vResultKeys.push_back(*pKP);
    }

}

void CML::Features::ORB::computeDescriptors(const GrayImage &image, List<Corner> &keypoints, List<Binary256Descriptor> &descriptors) {

    #pragma omp single
    {
        descriptors.clear();
        descriptors.resize(keypoints.size());
    }

    #if CML_USE_OPENMP
    #pragma omp  for schedule(static)
    #endif
    for (size_t i = 0; i < keypoints.size(); i++) {
        descriptors[i] = computeDescriptor(keypoints[i], image);
    }
}

CML::Binary256Descriptor CML::Features::ORB::computeDescriptor(const Corner &corner, const GrayImage &frame) {


#define GET_VALUE(i, j) (*(image_center + ia_y[i*32 + j]*stride_image + ia_x[i*32 + j]) < *(image_center + ib_y[i*32 + j]*stride_image + ib_x[i*32 + j])) << j

    unsigned char *image_src = (unsigned char*)frame.data();
    DistortedVector2d point = corner.point(0);

    int stride_image = frame.getWidth();


    float cos_angle = cos(corner.angle());
    float sin_angle = sin(corner.angle());
    unsigned char *image_center = image_src + (int) point.y() * stride_image + (int) point.x();
    // N_DIM_BINARYDESCRIPTOR / SIZE_BITS_HAMING = 4
    alignas(64) int32_t ia_x[256];
    alignas(64) int32_t ia_y[256];
    alignas(64) int32_t ib_x[256];
    alignas(64) int32_t ib_y[256];

    // todo : use sse here
    for (int i = 0; i < 256; i++) {
        float x_a = bit_pattern_31_[i * 4 + 0];
        float y_a = bit_pattern_31_[i * 4 + 1];
        float x_b = bit_pattern_31_[i * 4 + 2];
        float y_b = bit_pattern_31_[i * 4 + 3];
        ia_x[i] = fastRound((x_a * cos_angle - y_a * sin_angle));
        ia_y[i] = fastRound((x_a * sin_angle + y_a * cos_angle));
        ib_x[i] = fastRound((x_b * cos_angle - y_b * sin_angle));
        ib_y[i] = fastRound((x_b * sin_angle + y_b * cos_angle));
    }


    alignas(64) int32_t f[8] = {0, 0, 0, 0, 0, 0, 0, 0};


    for (int py = 0; py < 32; py++) {
        for (int px = 0; px < 8; px++) {
            f[px] |= GET_VALUE(px, py);
        }
    }

    /*
    f[0] |= GET_VALUE(0, 0);
    f[0] |= GET_VALUE(0, 1);
    f[0] |= GET_VALUE(0, 2);
    f[0] |= GET_VALUE(0, 3);
    f[0] |= GET_VALUE(0, 4);
    f[0] |= GET_VALUE(0, 5);
    f[0] |= GET_VALUE(0, 6);
    f[0] |= GET_VALUE(0, 7);
    f[0] |= GET_VALUE(0, 8);
    f[0] |= GET_VALUE(0, 9);
    f[0] |= GET_VALUE(0, 10);
    f[0] |= GET_VALUE(0, 11);
    f[0] |= GET_VALUE(0, 12);
    f[0] |= GET_VALUE(0, 13);
    f[0] |= GET_VALUE(0, 14);
    f[0] |= GET_VALUE(0, 15);
    f[0] |= GET_VALUE(0, 16);
    f[0] |= GET_VALUE(0, 17);
    f[0] |= GET_VALUE(0, 18);
    f[0] |= GET_VALUE(0, 19);
    f[0] |= GET_VALUE(0, 20);
    f[0] |= GET_VALUE(0, 21);
    f[0] |= GET_VALUE(0, 22);
    f[0] |= GET_VALUE(0, 23);
    f[0] |= GET_VALUE(0, 24);
    f[0] |= GET_VALUE(0, 25);
    f[0] |= GET_VALUE(0, 26);
    f[0] |= GET_VALUE(0, 27);
    f[0] |= GET_VALUE(0, 28);
    f[0] |= GET_VALUE(0, 29);
    f[0] |= GET_VALUE(0, 30);
    f[0] |= GET_VALUE(0, 31);

    f[1] |= GET_VALUE(1, 0);
    f[1] |= GET_VALUE(1, 1);
    f[1] |= GET_VALUE(1, 2);
    f[1] |= GET_VALUE(1, 3);
    f[1] |= GET_VALUE(1, 4);
    f[1] |= GET_VALUE(1, 5);
    f[1] |= GET_VALUE(1, 6);
    f[1] |= GET_VALUE(1, 7);
    f[1] |= GET_VALUE(1, 8);
    f[1] |= GET_VALUE(1, 9);
    f[1] |= GET_VALUE(1, 10);
    f[1] |= GET_VALUE(1, 11);
    f[1] |= GET_VALUE(1, 12);
    f[1] |= GET_VALUE(1, 13);
    f[1] |= GET_VALUE(1, 14);
    f[1] |= GET_VALUE(1, 15);
    f[1] |= GET_VALUE(1, 16);
    f[1] |= GET_VALUE(1, 17);
    f[1] |= GET_VALUE(1, 18);
    f[1] |= GET_VALUE(1, 19);
    f[1] |= GET_VALUE(1, 20);
    f[1] |= GET_VALUE(1, 21);
    f[1] |= GET_VALUE(1, 22);
    f[1] |= GET_VALUE(1, 23);
    f[1] |= GET_VALUE(1, 24);
    f[1] |= GET_VALUE(1, 25);
    f[1] |= GET_VALUE(1, 26);
    f[1] |= GET_VALUE(1, 27);
    f[1] |= GET_VALUE(1, 28);
    f[1] |= GET_VALUE(1, 29);
    f[1] |= GET_VALUE(1, 30);
    f[1] |= GET_VALUE(1, 31);

    f[2] |= GET_VALUE(2, 0);
    f[2] |= GET_VALUE(2, 1);
    f[2] |= GET_VALUE(2, 2);
    f[2] |= GET_VALUE(2, 3);
    f[2] |= GET_VALUE(2, 4);
    f[2] |= GET_VALUE(2, 5);
    f[2] |= GET_VALUE(2, 6);
    f[2] |= GET_VALUE(2, 7);
    f[2] |= GET_VALUE(2, 8);
    f[2] |= GET_VALUE(2, 9);
    f[2] |= GET_VALUE(2, 10);
    f[2] |= GET_VALUE(2, 11);
    f[2] |= GET_VALUE(2, 12);
    f[2] |= GET_VALUE(2, 13);
    f[2] |= GET_VALUE(2, 14);
    f[2] |= GET_VALUE(2, 15);
    f[2] |= GET_VALUE(2, 16);
    f[2] |= GET_VALUE(2, 17);
    f[2] |= GET_VALUE(2, 18);
    f[2] |= GET_VALUE(2, 19);
    f[2] |= GET_VALUE(2, 20);
    f[2] |= GET_VALUE(2, 21);
    f[2] |= GET_VALUE(2, 22);
    f[2] |= GET_VALUE(2, 23);
    f[2] |= GET_VALUE(2, 24);
    f[2] |= GET_VALUE(2, 25);
    f[2] |= GET_VALUE(2, 26);
    f[2] |= GET_VALUE(2, 27);
    f[2] |= GET_VALUE(2, 28);
    f[2] |= GET_VALUE(2, 29);
    f[2] |= GET_VALUE(2, 30);
    f[2] |= GET_VALUE(2, 31);

    f[3] |= GET_VALUE(3, 0);
    f[3] |= GET_VALUE(3, 1);
    f[3] |= GET_VALUE(3, 2);
    f[3] |= GET_VALUE(3, 3);
    f[3] |= GET_VALUE(3, 4);
    f[3] |= GET_VALUE(3, 5);
    f[3] |= GET_VALUE(3, 6);
    f[3] |= GET_VALUE(3, 7);
    f[3] |= GET_VALUE(3, 8);
    f[3] |= GET_VALUE(3, 9);
    f[3] |= GET_VALUE(3, 10);
    f[3] |= GET_VALUE(3, 11);
    f[3] |= GET_VALUE(3, 12);
    f[3] |= GET_VALUE(3, 13);
    f[3] |= GET_VALUE(3, 14);
    f[3] |= GET_VALUE(3, 15);
    f[3] |= GET_VALUE(3, 16);
    f[3] |= GET_VALUE(3, 17);
    f[3] |= GET_VALUE(3, 18);
    f[3] |= GET_VALUE(3, 19);
    f[3] |= GET_VALUE(3, 20);
    f[3] |= GET_VALUE(3, 21);
    f[3] |= GET_VALUE(3, 22);
    f[3] |= GET_VALUE(3, 23);
    f[3] |= GET_VALUE(3, 24);
    f[3] |= GET_VALUE(3, 25);
    f[3] |= GET_VALUE(3, 26);
    f[3] |= GET_VALUE(3, 27);
    f[3] |= GET_VALUE(3, 28);
    f[3] |= GET_VALUE(3, 29);
    f[3] |= GET_VALUE(3, 30);
    f[3] |= GET_VALUE(3, 31);

    f[4] |= GET_VALUE(4, 0);
    f[4] |= GET_VALUE(4, 1);
    f[4] |= GET_VALUE(4, 2);
    f[4] |= GET_VALUE(4, 3);
    f[4] |= GET_VALUE(4, 4);
    f[4] |= GET_VALUE(4, 5);
    f[4] |= GET_VALUE(4, 6);
    f[4] |= GET_VALUE(4, 7);
    f[4] |= GET_VALUE(4, 8);
    f[4] |= GET_VALUE(4, 9);
    f[4] |= GET_VALUE(4, 10);
    f[4] |= GET_VALUE(4, 11);
    f[4] |= GET_VALUE(4, 12);
    f[4] |= GET_VALUE(4, 13);
    f[4] |= GET_VALUE(4, 14);
    f[4] |= GET_VALUE(4, 15);
    f[4] |= GET_VALUE(4, 16);
    f[4] |= GET_VALUE(4, 17);
    f[4] |= GET_VALUE(4, 18);
    f[4] |= GET_VALUE(4, 19);
    f[4] |= GET_VALUE(4, 20);
    f[4] |= GET_VALUE(4, 21);
    f[4] |= GET_VALUE(4, 22);
    f[4] |= GET_VALUE(4, 23);
    f[4] |= GET_VALUE(4, 24);
    f[4] |= GET_VALUE(4, 25);
    f[4] |= GET_VALUE(4, 26);
    f[4] |= GET_VALUE(4, 27);
    f[4] |= GET_VALUE(4, 28);
    f[4] |= GET_VALUE(4, 29);
    f[4] |= GET_VALUE(4, 30);
    f[4] |= GET_VALUE(4, 31);

    f[5] |= GET_VALUE(5, 0);
    f[5] |= GET_VALUE(5, 1);
    f[5] |= GET_VALUE(5, 2);
    f[5] |= GET_VALUE(5, 3);
    f[5] |= GET_VALUE(5, 4);
    f[5] |= GET_VALUE(5, 5);
    f[5] |= GET_VALUE(5, 6);
    f[5] |= GET_VALUE(5, 7);
    f[5] |= GET_VALUE(5, 8);
    f[5] |= GET_VALUE(5, 9);
    f[5] |= GET_VALUE(5, 10);
    f[5] |= GET_VALUE(5, 11);
    f[5] |= GET_VALUE(5, 12);
    f[5] |= GET_VALUE(5, 13);
    f[5] |= GET_VALUE(5, 14);
    f[5] |= GET_VALUE(5, 15);
    f[5] |= GET_VALUE(5, 16);
    f[5] |= GET_VALUE(5, 17);
    f[5] |= GET_VALUE(5, 18);
    f[5] |= GET_VALUE(5, 19);
    f[5] |= GET_VALUE(5, 20);
    f[5] |= GET_VALUE(5, 21);
    f[5] |= GET_VALUE(5, 22);
    f[5] |= GET_VALUE(5, 23);
    f[5] |= GET_VALUE(5, 24);
    f[5] |= GET_VALUE(5, 25);
    f[5] |= GET_VALUE(5, 26);
    f[5] |= GET_VALUE(5, 27);
    f[5] |= GET_VALUE(5, 28);
    f[5] |= GET_VALUE(5, 29);
    f[5] |= GET_VALUE(5, 30);
    f[5] |= GET_VALUE(5, 31);

    f[6] |= GET_VALUE(6, 0);
    f[6] |= GET_VALUE(6, 1);
    f[6] |= GET_VALUE(6, 2);
    f[6] |= GET_VALUE(6, 3);
    f[6] |= GET_VALUE(6, 4);
    f[6] |= GET_VALUE(6, 5);
    f[6] |= GET_VALUE(6, 6);
    f[6] |= GET_VALUE(6, 7);
    f[6] |= GET_VALUE(6, 8);
    f[6] |= GET_VALUE(6, 9);
    f[6] |= GET_VALUE(6, 10);
    f[6] |= GET_VALUE(6, 11);
    f[6] |= GET_VALUE(6, 12);
    f[6] |= GET_VALUE(6, 13);
    f[6] |= GET_VALUE(6, 14);
    f[6] |= GET_VALUE(6, 15);
    f[6] |= GET_VALUE(6, 16);
    f[6] |= GET_VALUE(6, 17);
    f[6] |= GET_VALUE(6, 18);
    f[6] |= GET_VALUE(6, 19);
    f[6] |= GET_VALUE(6, 20);
    f[6] |= GET_VALUE(6, 21);
    f[6] |= GET_VALUE(6, 22);
    f[6] |= GET_VALUE(6, 23);
    f[6] |= GET_VALUE(6, 24);
    f[6] |= GET_VALUE(6, 25);
    f[6] |= GET_VALUE(6, 26);
    f[6] |= GET_VALUE(6, 27);
    f[6] |= GET_VALUE(6, 28);
    f[6] |= GET_VALUE(6, 29);
    f[6] |= GET_VALUE(6, 30);
    f[6] |= GET_VALUE(6, 31);

    f[7] |= GET_VALUE(7, 0);
    f[7] |= GET_VALUE(7, 1);
    f[7] |= GET_VALUE(7, 2);
    f[7] |= GET_VALUE(7, 3);
    f[7] |= GET_VALUE(7, 4);
    f[7] |= GET_VALUE(7, 5);
    f[7] |= GET_VALUE(7, 6);
    f[7] |= GET_VALUE(7, 7);
    f[7] |= GET_VALUE(7, 8);
    f[7] |= GET_VALUE(7, 9);
    f[7] |= GET_VALUE(7, 10);
    f[7] |= GET_VALUE(7, 11);
    f[7] |= GET_VALUE(7, 12);
    f[7] |= GET_VALUE(7, 13);
    f[7] |= GET_VALUE(7, 14);
    f[7] |= GET_VALUE(7, 15);
    f[7] |= GET_VALUE(7, 16);
    f[7] |= GET_VALUE(7, 17);
    f[7] |= GET_VALUE(7, 18);
    f[7] |= GET_VALUE(7, 19);
    f[7] |= GET_VALUE(7, 20);
    f[7] |= GET_VALUE(7, 21);
    f[7] |= GET_VALUE(7, 22);
    f[7] |= GET_VALUE(7, 23);
    f[7] |= GET_VALUE(7, 24);
    f[7] |= GET_VALUE(7, 25);
    f[7] |= GET_VALUE(7, 26);
    f[7] |= GET_VALUE(7, 27);
    f[7] |= GET_VALUE(7, 28);
    f[7] |= GET_VALUE(7, 29);
    f[7] |= GET_VALUE(7, 30);
    f[7] |= GET_VALUE(7, 31);
     */

    return Binary256Descriptor((uint64_t*)f);

#undef GET_VALUE


}

void CML::Features::ORB::computeOrientation(const GrayImage &image, List<Corner> &keypoints, const List<int> &umax) {

    for (Corner &pt : keypoints) {

        int m_01 = 0, m_10 = 0;


        // Treat the center line differently, v=0
        for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
            m_10 += u * image(pt.x() + u, pt.y());

        // Go line by line in the circuI853lar patch
        for (int v = 1; v <= HALF_PATCH_SIZE; ++v)
        {
            // Proceed over the two lines
            int v_sum = 0;
            int d = umax[v];
            for (int u = -d; u <= d; ++u)
            {
                int val_plus = image(pt.x() + u, pt.y() + v), val_minus = image(pt.x() + u, pt.y() - v);
                v_sum += (val_plus - val_minus);
                m_10 += u * (val_plus + val_minus);
            }
            m_01 += v * v_sum;
        }

        scalar_t angle = atan2((float)m_01, (float)m_10);
        pt.setAngle(angle);


    }

}

void CML::Features::ORB::ExtractorNode::divideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4) {

    const int halfX = ceil(static_cast<float>(UR.x()-UL.x())/2);
    const int halfY = ceil(static_cast<float>(BR.y()-UL.y())/2);

    //Define boundaries of childs
    n1.UL = UL;
    n1.UR = Vector2i(UL.x()+halfX,UL.y());
    n1.BL = Vector2i(UL.x(),UL.y()+halfY);
    n1.BR = Vector2i(UL.x()+halfX,UL.y()+halfY);
    n1.vKeys.reserve(vKeys.size());

    n2.UL = n1.UR;
    n2.UR = UR;
    n2.BL = n1.BR;
    n2.BR = Vector2i(UR.x(),UL.y()+halfY);
    n2.vKeys.reserve(vKeys.size());

    n3.UL = n1.BL;
    n3.UR = n1.BR;
    n3.BL = BL;
    n3.BR = Vector2i(n1.BR.x(),BL.y());
    n3.vKeys.reserve(vKeys.size());

    n4.UL = n3.UR;
    n4.UR = n2.BR;
    n4.BL = n3.BR;
    n4.BR = BR;
    n4.vKeys.reserve(vKeys.size());

    //Associate points to childs
    for(size_t i=0;i<vKeys.size();i++)
    {
        const Corner &kp = vKeys[i];
        if(kp.x()<n1.UR.x())
        {
            if(kp.y()<n1.BR.y())
                n1.vKeys.push_back(kp);
            else
                n3.vKeys.push_back(kp);
        }
        else if(kp.y()<n1.BR.y())
            n2.vKeys.push_back(kp);
        else
            n4.vKeys.push_back(kp);
    }

    if(n1.vKeys.size()==1)
        n1.bNoMore = true;
    if(n2.vKeys.size()==1)
        n2.bNoMore = true;
    if(n3.vKeys.size()==1)
        n3.bNoMore = true;
    if(n4.vKeys.size()==1)
        n4.bNoMore = true;

}
