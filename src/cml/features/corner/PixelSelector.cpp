#include "cml/features/corner/PixelSelector.h"

// parameters controlling pixel selection
float setting_minGradHistCut = 0.5;
float setting_minGradHistAdd = 7;
float setting_gradDownweightPerLevel = 0.75;
bool  setting_selectDirectionDistribution = true;


CML::Features::PixelSelector::PixelSelector(Ptr<AbstractFunction, Nullable> parent, int w, int h) : AbstractFunction(parent)
{
    randomPattern = new unsigned char[w*h];
    for(int i=0; i<w * h; i++) randomPattern[i] = myRand() & 0xFF;
    currentPotential=3;


    gradHist = new int[100*(1+w/32)*(1+h/32)];
    ths = new float[(w/32)*(h/32)+100];
    thsSmoothed = new float[(w/32)*(h/32)+100];

}

CML::Features::PixelSelector::~PixelSelector()
{
    delete[] randomPattern;
    delete[] gradHist;
    delete[] ths;
    delete[] thsSmoothed;
}

int computeHistQuantil(const int* hist, float below)
{
    int th = lroundf(static_cast<float>(hist[0])*below);
    for(int i=0;i<90;i++)
    {
        th -= hist[i+1];
        if(th<0) return i;
    }
    return 90;
}


void CML::Features::PixelSelector::makeHists(const CaptureImage &cp)
{
    // float * mapmax0 = fh->absSquaredGrad[0];
    int w = cp.getGrayImage(0).getWidth();
    int h = cp.getGrayImage(0).getHeight();

    for (int i = 0; i < (w/32)*(h/32)+100; i++) {
        ths[i] = 0;
    }
    for (int i = 0; i < (w/32)*(h/32)+100; i++) {
        thsSmoothed[i] = 0;
    }
    for (int i = 0; i < 100*(1+w/32)*(1+h/32); i++) {
        gradHist[i] = 0;
    }

    int w32 = w/32;
    int h32 = h/32;
    thsStep = w32;

    for(int y=0;y<h32;y++)
        for(int x=0;x<w32;x++)
        {
            // float* map0 = mapmax0+32*x+32*y*w;
            int* hist0 = gradHist;// + 50*(x+y*w32);
            memset(hist0,0,sizeof(int)*50);

            for(int j=0;j<32;j++) for(int i=0;i<32;i++)
                {
                    int it = i+32*x;
                    int jt = j+32*y;
                    if(it>w-2 || jt>h-2 || it<1 || jt<1) continue;

                    int g = static_cast<int>(sqrtf(cp.getWeightedGradientNorm(0).get(32 * x + i, 32 * y + j)));
                    // int g = sqrtf(map0[i+j*w]);

                    if(g>48) g=48;
                    //assertThrow(g >= 0, "Weghted gradient norm is not correct !");
                    if (g>=0 && std::isfinite(g)) {
                        hist0[g + 1]++;
                        hist0[0]++;
                    }
                }

            ths[x+y*w32] = static_cast<float>(computeHistQuantil(hist0,setting_minGradHistCut)) + setting_minGradHistAdd;
        }

    for(int y=0;y<h32;y++)
        for(int x=0;x<w32;x++)
        {
            float sum=0,num=0;
            if(x>0)
            {
                if(y>0) 	{num++; 	sum+=ths[x-1+(y-1)*w32];}
                if(y<h32-1) {num++; 	sum+=ths[x-1+(y+1)*w32];}
                num++; sum+=ths[x-1+(y)*w32];
            }

            if(x<w32-1)
            {
                if(y>0) 	{num++; 	sum+=ths[x+1+(y-1)*w32];}
                if(y<h32-1) {num++; 	sum+=ths[x+1+(y+1)*w32];}
                num++; sum+=ths[x+1+(y)*w32];
            }

            if(y>0) 	{num++; 	sum+=ths[x+(y-1)*w32];}
            if(y<h32-1) {num++; 	sum+=ths[x+(y+1)*w32];}
            num++; sum+=ths[x+y*w32];

            thsSmoothed[x+y*w32] = (sum/num) * (sum/num);

        }
    
}


int CML::Features::PixelSelector::makeMaps(
        const CaptureImage &cp,
        float* map_out, float density, int recursionsLeft, float thFactor)
{
    float numHave=0;
    float numWant=density;
    float quotia;
    int idealPotential = currentPotential;

    
    {




        // the number of selected pixels behaves approximately as
        // K / (pot+1)^2, where K is a scene-dependent constant.
        // we will allow sub-selecting pixels by up to a quotia of 0.25, otherwise we will re-select.
        makeHists(cp);

        // select!
        Eigen::Vector3i n = this->select(cp, map_out,currentPotential, thFactor);

        // sub-select!
        numHave = static_cast<float>(n[0] + n[1] + n[2]);
        quotia = numWant / numHave;

        // by default we want to over-sample by 40% just to be sure.
        float K = numHave * static_cast<float>((currentPotential+1) * (currentPotential+1));
        idealPotential = static_cast<int>(sqrtf(K/numWant)) - 1;	// round down.
        if(idealPotential<1) idealPotential=1;

        if( recursionsLeft>0 && quotia > 1.25 && currentPotential>1)
        {
            //re-sample to get more points!
            // potential needs to be smaller
            if(idealPotential>=currentPotential)
                idealPotential = currentPotential-1;

            //		printf("PixelSelector: have %.2f%%, need %.2f%%. RESAMPLE with pot %d -> %d.\n",
            //				100*numHave/(float)(framePyramid.getMatrix(0).getWidth();*framePyramid.getMatrix(0).getHeight();),
            //				100*numWant/(float)(framePyramid.getMatrix(0).getWidth();*framePyramid.getMatrix(0).getHeight();),
            //				currentPotential,
            //				idealPotential);
            currentPotential = idealPotential;
            return makeMaps(cp, map_out, density, recursionsLeft-1,thFactor);
        }
        else if(recursionsLeft>0 && quotia < 0.25)
        {
            // re-sample to get less points!

            if(idealPotential<=currentPotential)
                idealPotential = currentPotential+1;

            //		printf("PixelSelector: have %.2f%%, need %.2f%%. RESAMPLE with pot %d -> %d.\n",
            //				100*numHave/(float)(framePyramid.getMatrix(0).getWidth();*framePyramid.getMatrix(0).getHeight();),
            //				100*numWant/(float)(framePyramid.getMatrix(0).getWidth();*framePyramid.getMatrix(0).getHeight();),
            //				currentPotential,
            //				idealPotential);
            currentPotential = idealPotential;
            return makeMaps(cp, map_out, density, recursionsLeft-1,thFactor);

        }
    }

    int numHaveSub = static_cast<int>(numHave);
    if(quotia < 0.95)
    {
        int wh= cp.getGrayImage(0).getWidth() * cp.getGrayImage(0).getHeight();
        int rn=0;
        unsigned char charTH = static_cast<unsigned char>(255.0f * quotia);
        for(int i=0;i<wh;i++)
        {
            if(map_out[i] != 0)
            {
                if(randomPattern[rn] > charTH )
                {
                    map_out[i]=0;
                    numHaveSub--;
                }
                rn++;
            }
        }
    }

//	printf("PixelSelector: have %.2f%%, need %.2f%%. KEEPCURR with pot %d -> %d. Subsampled to %.2f%%\n",
//			100*numHave/(float)(framePyramid.getMatrix(0).getWidth();*framePyramid.getMatrix(0).getHeight();),
//			100*numWant/(float)(framePyramid.getMatrix(0).getWidth();*framePyramid.getMatrix(0).getHeight();),
//			currentPotential,
//			idealPotential,
//			100*numHaveSub/(float)(framePyramid.getMatrix(0).getWidth();*framePyramid.getMatrix(0).getHeight();));
    currentPotential = idealPotential;

    return numHaveSub;
}



Eigen::Vector3i CML::Features::PixelSelector::select(const CaptureImage &cp, float* map_out, int pot, float thFactor)
{
/*
    Eigen::Vector3f const * const map0 = fh->dI;

    float * mapmax0 = fh->absSquaredGrad[0];
    float * mapmax1 = fh->absSquaredGrad[1];
    float * mapmax2 = fh->absSquaredGrad[2];
*/

    int w = cp.getGrayImage(0).getWidth();
    int h = cp.getGrayImage(0).getHeight();


    const Eigen::Vector2f directions[16] = {
            Eigen::Vector2f(0,    1.0000),
            Eigen::Vector2f(0.3827,    0.9239),
            Eigen::Vector2f(0.1951,    0.9808),
            Eigen::Vector2f(0.9239,    0.3827),
            Eigen::Vector2f(0.7071,    0.7071),
            Eigen::Vector2f(0.3827,   -0.9239),
            Eigen::Vector2f(0.8315,    0.5556),
            Eigen::Vector2f(0.8315,   -0.5556),
            Eigen::Vector2f(0.5556,   -0.8315),
            Eigen::Vector2f(0.9808,    0.1951),
            Eigen::Vector2f(0.9239,   -0.3827),
            Eigen::Vector2f(0.7071,   -0.7071),
            Eigen::Vector2f(0.5556,    0.8315),
            Eigen::Vector2f(0.9808,   -0.1951),
            Eigen::Vector2f(1.0000,    0.0000),
            Eigen::Vector2f(0.1951,   -0.9808)};

    for (size_t i = 0; i < w * h; i++) {
        map_out[i] = 0;
    }



    float dw1 = setting_gradDownweightPerLevel;
    float dw2 = dw1*dw1;

    assertDeterministic("thsStep", thsStep);

    int n3=0, n2=0, n4=0;
    for(int y4=0;y4<h;y4+=(4*pot)) for(int x4=0;x4<w;x4+=(4*pot))
        {
            int my3 = std::min((4*pot), h-y4);
            int mx3 = std::min((4*pot), w-x4);
            int bestIdx4=-1; float bestVal4=0;
            Eigen::Vector2f dir4 = directions[randomPattern[n2] & 0xF];
            for(int y3=0;y3<my3;y3+=(2*pot)) for(int x3=0;x3<mx3;x3+=(2*pot))
                {
                    int x34 = x3+x4;
                    int y34 = y3+y4;
                    int my2 = std::min((2*pot), h-y34);
                    int mx2 = std::min((2*pot), w-x34);
                    int bestIdx3=-1; float bestVal3=0;
                    Eigen::Vector2f dir3 = directions[randomPattern[n2] & 0xF];
                    for(int y2=0;y2<my2;y2+=pot) for(int x2=0;x2<mx2;x2+=pot)
                        {
                            int x234 = x2+x34;
                            int y234 = y2+y34;
                            int my1 = std::min(pot, h-y234);
                            int mx1 = std::min(pot, w-x234);
                            int bestIdx2=-1; float bestVal2=0;
                            Eigen::Vector2f dir2 = directions[randomPattern[n2] & 0xF];
                            for(int y1=0;y1<my1;y1+=1) for(int x1=0;x1<mx1;x1+=1)
                                {
                                    assertThrow(x1+x234 < w, "Out of range");
                                    assertThrow(y1+y234 < h, "Out of range");
                                    int idx = x1+x234 + w*(y1+y234);
                                    int xf = x1+x234;
                                    int yf = y1+y234;

                                    if(xf<4 || xf>=w-5 || yf<4 || yf>h-4) continue;

                                    float pixelTH0 = thsSmoothed[(xf>>5) + (yf>>5) * thsStep];
                                    float pixelTH1 = pixelTH0*dw1;
                                    float pixelTH2 = pixelTH1*dw2;

                                    float ag0 = cp.getWeightedGradientNorm(0).get(x1 + x234, y1 + y234);
                                    if(ag0 > pixelTH0*thFactor)
                                    {
                                        Eigen::Vector2f ag0d = cp.getDerivativeImage(0).get(x1 + x234, y1 + y234).tail<2>().cast<float>();
                                        float dirNorm = fabsf((float)(ag0d.dot(dir2)));
                                        if(!setting_selectDirectionDistribution) dirNorm = ag0;

                                        if(dirNorm > bestVal2)
                                        { bestVal2 = dirNorm; bestIdx2 = idx; bestIdx3 = -2; bestIdx4 = -2;}
                                    }
                                    if(bestIdx3==-2) continue;

                                    float ag1 = cp.getWeightedGradientNorm(1).get(static_cast<int>(static_cast<float>(xf)*0.5f+0.25f),static_cast<int>(static_cast<float>(yf)*0.5f+0.25f));
                                    if(ag1 > pixelTH1*thFactor)
                                    {
                                        Eigen::Vector2f ag0d = cp.getDerivativeImage(0).get(x1 + x234, y1 + y234).tail<2>().cast<float>();
                                        float dirNorm = fabsf((float)(ag0d.dot(dir3)));
                                        if(!setting_selectDirectionDistribution) dirNorm = ag1;

                                        if(dirNorm > bestVal3)
                                        { bestVal3 = dirNorm; bestIdx3 = idx; bestIdx4 = -2;}
                                    }
                                    if(bestIdx4==-2) continue;

                                    float ag2 = cp.getWeightedGradientNorm(2).get(static_cast<int>(static_cast<float>(xf)*0.25f+0.125),static_cast<int>(static_cast<float>(yf)*0.25f+0.125));
                                    if(ag2 > pixelTH2*thFactor)
                                    {
                                        Eigen::Vector2f ag0d = cp.getDerivativeImage(0).get(x1 + x234, y1 + y234).tail<2>().cast<float>();
                                        float dirNorm = fabsf((float)(ag0d.dot(dir4)));
                                        if(!setting_selectDirectionDistribution) dirNorm = ag2;

                                        if(dirNorm > bestVal4)
                                        { bestVal4 = dirNorm; bestIdx4 = idx; }
                                    }

                                }

                            if(bestIdx2>0)
                            {
                                map_out[bestIdx2] = 1;
                                bestVal3 = 1e10;
                                n2++;
                            }
                        }

                    if(bestIdx3>0)
                    {
                        map_out[bestIdx3] = 2;
                        bestVal4 = 1e10;
                        n3++;
                    }
                }

            if(bestIdx4>0)
            {
                map_out[bestIdx4] = 4;
                n4++;
            }
        }


    assertDeterministic("n2", n2);
    assertDeterministic("n3", n3);
    assertDeterministic("n4", n4);


    return {n2,n3,n4};
}

void CML::Features::PixelSelector::compute(const CaptureImage &cp, List<Corner> &corners, List<float> &types, float density, int recursionsLeft, float thFactor) {

    Array2D<float> output(cp.getWidth(0), cp.getHeight(0), 0.0f);
    makeMaps(cp, output.data(), density, recursionsLeft, thFactor);

    // #pragma omp  for collapse(2) // todo : thread safe list
    for (int i = 32; i < cp.getWidth(0) - 32; i++) {
        for (int j = 32; j < cp.getHeight(0) - 32; j++) {
            if (output(i, j) != 0 && cp.getDerivativeImage(0).get(i, j).allFinite()) {
                assertDeterministicMsg("PS extracted : " + std::to_string(i) + " " + std::to_string(j));
                corners.emplace_back(DistortedVector2d(i, j));
                types.emplace_back(output(i, j));
            }
        }
    }

}

int CML::Features::PixelSelector::makeMaps(const CaptureImage &cp, Array2D<float>& map_out, float density, int recursionsLeft, float thFactor) {

    map_out = Array2D<float>(cp.getWidth(0), cp.getHeight(0), 0.0f);
    int res = makeMaps(cp, map_out.data(), density, recursionsLeft, thFactor);
    for (int y = 0; y < cp.getHeight(0); y++) {
        for (int x = 0; x < cp.getWidth(0); x++) {
            if (!cp.getGrayImage(0).goodPosition(x, y)) {
                map_out(x, y) = 0;
            }
        }
    }
    return res;

}

int CML::Features::PixelSelector::makePixelStatus(const CaptureImage &cp, int lvl, Array2D<bool>& map, float desiredDensity, int &sparsityFactor, int recsLeft, float THFac) {

    map = Array2D<bool>(cp.getWidth(lvl), cp.getHeight(lvl), false);
    return makePixelStatus(cp, lvl, map.data(), desiredDensity, sparsityFactor, recsLeft, THFac);

}

template<int pot> int CML::Features::PixelSelector::gridMaxSelection(const CaptureImage &cp, int lvl, bool* map_out, float THFac)
{

    int w = cp.getWidth(lvl);
    int h = cp.getHeight(lvl);

    memset(map_out, 0, sizeof(bool)*w*h);

    int numGood = 0;
    for(int y=1;y<h-pot;y+=pot)
    {
        for(int x=1;x<w-pot;x+=pot)
        {
            int bestXXID = -1;
            int bestYYID = -1;
            int bestXYID = -1;
            int bestYXID = -1;

            float bestXX=0, bestYY=0, bestXY=0, bestYX=0;

            //Eigen::Vector3f* grads0 = grads+x+y*w;
            for(int dx=0;dx<pot;dx++)
                for(int dy=0;dy<pot;dy++)
                {
                    int idx = dx+dy*w;
                    auto g = cp.getDerivativeImage(lvl).get(x + dx, y + dy);
                    float sqgd = g.tail<2>().squaredNorm();
                    //Eigen::Vector3f g=grads0[idx];
                    //float sqgd = g.tail<2>().squaredNorm();
                    float TH = THFac*minUseGrad_pixsel * (0.75f);

                    if(sqgd > TH*TH)
                    {
                        float agx = fabsf((float)g[1]);
                        if(agx > bestXX) {bestXX=agx; bestXXID=idx;}

                        float agy = fabsf((float)g[2]);
                        if(agy > bestYY) {bestYY=agy; bestYYID=idx;}

                        float gxpy = fabsf((float)(g[1]-g[2]));
                        if(gxpy > bestXY) {bestXY=gxpy; bestXYID=idx;}

                        float gxmy = fabsf((float)(g[1]+g[2]));
                        if(gxmy > bestYX) {bestYX=gxmy; bestYXID=idx;}
                    }
                }

            bool* map0 = map_out+x+y*w;

            if(bestXXID>=0)
            {
                if(!map0[bestXXID])
                    numGood++;
                map0[bestXXID] = true;

            }
            if(bestYYID>=0)
            {
                if(!map0[bestYYID])
                    numGood++;
                map0[bestYYID] = true;

            }
            if(bestXYID>=0)
            {
                if(!map0[bestXYID])
                    numGood++;
                map0[bestXYID] = true;

            }
            if(bestYXID>=0)
            {
                if(!map0[bestYXID])
                    numGood++;
                map0[bestYXID] = true;

            }
        }
    }

    return numGood;
}


int CML::Features::PixelSelector::gridMaxSelection(const CaptureImage &cp, int lvl, bool* map_out, int pot, float THFac) const
{
    int w = cp.getWidth(lvl);
    int h = cp.getHeight(lvl);

    memset(map_out, 0, sizeof(bool)*w*h);

    int numGood = 0;
    for(int y=1;y<h-pot;y+=pot)
    {
        for(int x=1;x<w-pot;x+=pot)
        {
            int bestXXID = -1;
            int bestYYID = -1;
            int bestXYID = -1;
            int bestYXID = -1;

            float bestXX=0, bestYY=0, bestXY=0, bestYX=0;

            // Eigen::Vector3f* grads0 = grads+x+y*w;
            for(int dx=0;dx<pot;dx++)
                for(int dy=0;dy<pot;dy++)
                {
                    int idx = dx+dy*w;
                    // Eigen::Vector3f g=grads0[idx];
                    // float sqgd = g.tail<2>().squaredNorm();
                    auto g = cp.getDerivativeImage(lvl).get(x + dx, y + dy);
                    float sqgd = g.tail<2>().squaredNorm();
                    float TH = THFac*minUseGrad_pixsel * (0.75f);

                    if(sqgd > TH*TH)
                    {
                        float agx = fabsf((float)g[1]);
                        if(agx > bestXX) {bestXX=agx; bestXXID=idx;}

                        float agy = fabsf((float)g[2]);
                        if(agy > bestYY) {bestYY=agy; bestYYID=idx;}

                        float gxpy = fabsf((float)(g[1]-g[2]));
                        if(gxpy > bestXY) {bestXY=gxpy; bestXYID=idx;}

                        float gxmy = fabsf((float)(g[1]+g[2]));
                        if(gxmy > bestYX) {bestYX=gxmy; bestYXID=idx;}
                    }
                }

            bool* map0 = map_out+x+y*w;

            if(bestXXID>=0)
            {
                if(!map0[bestXXID])
                    numGood++;
                map0[bestXXID] = true;

            }
            if(bestYYID>=0)
            {
                if(!map0[bestYYID])
                    numGood++;
                map0[bestYYID] = true;

            }
            if(bestXYID>=0)
            {
                if(!map0[bestXYID])
                    numGood++;
                map0[bestXYID] = true;

            }
            if(bestYXID>=0)
            {
                if(!map0[bestYXID])
                    numGood++;
                map0[bestYXID] = true;

            }
        }
    }

    return numGood;
}

int CML::Features::PixelSelector::makePixelStatus(const CaptureImage &cp, int lvl, bool *map, float desiredDensity, int &sparsityFactor, int recsLeft, float THFac) {
    if(sparsityFactor < 1) sparsityFactor = 1;

    int numGoodPoints;


    if(sparsityFactor==1) numGoodPoints = gridMaxSelection<1>(cp, lvl, map, THFac);
    else if(sparsityFactor==2) numGoodPoints = gridMaxSelection<2>(cp, lvl, map, THFac);
    else if(sparsityFactor==3) numGoodPoints = gridMaxSelection<3>(cp, lvl, map, THFac);
    else if(sparsityFactor==4) numGoodPoints = gridMaxSelection<4>(cp, lvl, map, THFac);
    else if(sparsityFactor==5) numGoodPoints = gridMaxSelection<5>(cp, lvl, map, THFac);
    else if(sparsityFactor==6) numGoodPoints = gridMaxSelection<6>(cp, lvl, map, THFac);
    else if(sparsityFactor==7) numGoodPoints = gridMaxSelection<7>(cp, lvl, map, THFac);
    else if(sparsityFactor==8) numGoodPoints = gridMaxSelection<8>(cp, lvl, map, THFac);
    else if(sparsityFactor==9) numGoodPoints = gridMaxSelection<9>(cp, lvl, map, THFac);
    else if(sparsityFactor==10) numGoodPoints = gridMaxSelection<10>(cp, lvl, map, THFac);
    else if(sparsityFactor==11) numGoodPoints = gridMaxSelection<11>(cp, lvl, map, THFac);
    else numGoodPoints = gridMaxSelection(cp, lvl, map, sparsityFactor, THFac);


    /*
     * #points is approximately proportional to sparsityFactor^2.
     */

    float quotia = static_cast<float>(numGoodPoints) / static_cast<float>(desiredDensity);

    int newSparsity = static_cast<int>((static_cast<float>(sparsityFactor) * sqrtf(quotia)) + 0.7f);


    if(newSparsity < 1) newSparsity=1;


    float oldTHFac = THFac;
    if(newSparsity==1 && sparsityFactor==1) THFac = 0.5;


    if((abs(newSparsity-sparsityFactor) < 1 && THFac==oldTHFac) ||
       ( quotia > 0.8 &&  1.0f / quotia > 0.8) ||
       recsLeft == 0)
    {

//		printf(" \n");
        //all good
        sparsityFactor = newSparsity;
        return numGoodPoints;
    }
    else
    {
//		printf(" -> re-evaluate! \n");
        // re-evaluate.
        sparsityFactor = newSparsity;
        return makePixelStatus(cp, lvl, map, desiredDensity, sparsityFactor, recsLeft-1, THFac);
    }
}
