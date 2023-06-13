#include <iostream>

#include "cml/features/corner/ORB.h"
#include "cml/image/Array2D.h"

int main(int argc, char **argv) {

    CML::initCML();

    std::cout << "Loading image..." << std::endl;
    CML::FloatImage image = CML::loadImage("../test.jpg", false).first.reduceByTwo();

    std::cout << "Loading ORB..." << std::endl;
    CML::Features::ORB orb(nullptr);
    orb.setNumFeatures(4000);

    std::cout << "Computing..." << std::endl;
    CML::Timer timer;
    timer.start();

    for (int i = 0; i < 100; i++) {
        std::cout << i << std::endl;
        orb.compute(image);
    }

    timer.stop();
    std::cout << "ORB: " << timer.getValue() / 100 << " seconds" << std::endl;



    return 0;
}