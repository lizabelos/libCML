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
            CML_LOG_IMPORTANT("Using cached ORB");
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

    compute(captureImage.getGrayImage(0));


    #pragma omp single
    if (mUseCache.b() && captureImage.getPath() != "") {
        std::string orbCachePath = captureImage.getPath() + ".orb" + std::to_string(mNumCorner.i());
        FILE *f = fopen(orbCachePath.c_str(), "wb");
        if (f != nullptr) {
            CML_LOG_IMPORTANT("Using cached ORB");
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

    //CML_LOG_IMPORTANT("Extracted " + std::to_string(mCorners.size()) + " orb coners");

}

void CML::Features::ORB::compute(const FloatImage &image) {
    image.castToUChar(mImages[0]);
    assertDeterministic("Image 0", mImages[0].eigenMatrix().sum());
    for (int i = 1; i < mNLevels.i(); i++) {
        int imgWidth = (float)image.getWidth() * mvInvScaleFactor[i];
        imgWidth = imgWidth / 16;
        imgWidth = imgWidth * 16;
        mImages[i - 1].resize(imgWidth, image.getHeight() * imgWidth / image.getWidth(), mImages[i]);
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
            mBluredImagesTmpD[level].castToUChar<unsigned char>(mBluredImages[level]);

            computeDescriptorsWithTransposedImage(mBluredImages[level], mAllKeypoints[level], desc);

            // mBluredImagesTmpD[level].transpose(mBluredImagesTmpE[level]);
            // mBluredImagesTmpE[level].castToUChar<unsigned char>(mBluredImages[level]);

            //mImages[level].cast<float>().convolution(mFilter, mBluredImages[level]);
        } else {
            mBluredImages[level].copyToThis(mImages[level]);

            // Compute the descriptors
            computeDescriptors(mBluredImages[level], mAllKeypoints[level], desc);

        }


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
                if (image.goodPosition(mAllKeypoints[level][j].x(), mAllKeypoints[level][j].y())) {
                    mCorners.emplace_back(mAllKeypoints[level][j]);
                    mDescriptors.emplace_back(desc[j]);
                }
            }
        }

    }
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
            CML_LOG_DEBUG("ORB computed " + std::to_string(keypoints.size()) + " at level " + std::to_string(level));
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

void CML::Features::ORB::computeDescriptorsWithTransposedImage(const GrayImage &image, List<Corner> &keypoints, List<Binary256Descriptor> &descriptors) {

#pragma omp single
    {
        descriptors.clear();
        descriptors.resize(keypoints.size());
    }

#if CML_USE_OPENMP
#pragma omp  for schedule(static)
#endif
    for (size_t i = 0; i < keypoints.size(); i++) {
        descriptors[i] = computeDescriptorWithTransposedImage(keypoints[i], image);
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


    for (int px = 0; px < 8; px++) {
        for (int py = 0; py < 32; py++) {
            f[px] |= GET_VALUE(px, py);
        }
    }

    return Binary256Descriptor((uint64_t*)f);

#undef GET_VALUE


}


CML::Binary256Descriptor CML::Features::ORB::computeDescriptorWithTransposedImage(const Corner &corner, const GrayImage &frame) {
    DistortedVector2d point = corner.point(0);

    int stride_image = frame.getHeight();

    float cos_angle = cos(corner.angle());
    float sin_angle = sin(corner.angle());

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

    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 32; j++) {
            unsigned char left = frame(point.y() + ia_y[i*32 + j], point.x() + ia_x[i*32 + j]);
            unsigned char right = frame(point.y() + ib_y[i*32 + j], point.x() + ib_x[i*32 + j]);
            f[i] |= (left < right) << j;
        }
    }

    return Binary256Descriptor((uint64_t*)f);

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
