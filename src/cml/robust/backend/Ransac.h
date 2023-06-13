#ifndef CML_RANSAC_H
#define CML_RANSAC_H

#include <cml/config.h>
#include <cml/base/AbstractFunction.h>
#include <cml/maths/Utils.h>
#include <random>

namespace CML::Robust {

    template <typename Model, typename Type> class Estimator {

    public:
        virtual List<Model> processModels(const List<Type> &data) = 0;

        virtual int getDataNumber() const = 0;

    };

    template <typename Model, typename Type> class LossFunction {

    public:
        virtual scalar_t processLoss(const Model &model, const Type &data) const = 0;

    };

    template <typename T>
    void processSubsample(const List<T> &data, List<T> &output, int samples) {

        assertThrow((int)data.size() >= samples, "Data size is lower than the number of desired samples");

        std::vector<size_t> indexes;
        indexes.resize(data.size());
        for (size_t i = 0; i < data.size(); i++) indexes[i] = i;

        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        shuffle (indexes.begin(), indexes.end(), std::default_random_engine(seed));

        for (int i = 0; i < samples; i++) {
            output.emplace_back(data[indexes[i]]);
        }

    }

    template <typename Model, typename Type> class RansacResult {

    public:
        bool haveBestModel = false;
        Model bestModel;
        scalar_t bestScore = std::numeric_limits<scalar_t>::max();

        List<Pair<Model, scalar_t>> models;

        List<bool> inliers;
    };

    template <typename Model, typename Type> class Ransac {

    public:
        explicit Ransac(Estimator<Model, Type> *estimator, LossFunction<Model, Type> *lossFunction, scalar_t threshold)
        : mEstimator(estimator), mLossFunction(lossFunction), mThreshold(threshold) {

        }

        RansacResult<Model, Type> compute(const List<Type> &data, int numIterations = 250, bool parallel = true) {


            RansacResult<Model, Type> result = RansacResult<Model, Type>();
            result.inliers.resize(data.size(), true);

            Mutex parallelMutex;

            if (parallel) {
                #if CML_USE_OPENMP
                #pragma omp  for schedule(dynamic)
                #endif
                for (size_t i = 0; i < numIterations; i++) {

                    List<Type> subsample;
                    processSubsample(data, subsample, mEstimator->getDataNumber());

                    List<Model> models = mEstimator->processModels(subsample);

                    for (const Model &model : models) {

                        List<bool> inliers;
                        scalar_t currentLoss = processLoss(data, model, inliers);

                        parallelMutex.lock();
                        if (currentLoss < result.bestScore) {
                            result.bestScore = currentLoss;
                            result.bestModel = model;
                            result.inliers = inliers;
                            result.haveBestModel = true;
                        }
                        parallelMutex.unlock();

                        result.models.emplace_back(Pair<Model, scalar_t>(model, currentLoss));
                    }

                }
            } else {
                for (size_t i = 0; i < numIterations; i++) {

                    List<Type> subsample;
                    processSubsample(data, subsample, mEstimator->getDataNumber());

                    List<Model> models = mEstimator->processModels(subsample);

                    for (const Model &model : models) {

                        List<bool> inliers;
                        scalar_t currentLoss = processLoss(data, model, inliers);

                        parallelMutex.lock();
                        if (currentLoss < result.bestScore) {
                            result.bestScore = currentLoss;
                            result.bestModel = model;
                            result.inliers = inliers;
                            result.haveBestModel = true;
                        }
                        parallelMutex.unlock();

                        result.models.emplace_back(Pair<Model, scalar_t>(model, currentLoss));
                    }

                }
            }

            return result;
        }

        scalar_t getThreshold() const {
            return mThreshold;
        }

        void setThreshold(scalar_t v) {
            mThreshold = v;
        }

    protected:
        virtual scalar_t processLoss(const List<Type> &data, const Model &model, List<bool> &inliers) {
            int currentLoss = 0;
            inliers.resize(data.size());

            for (size_t j = 0; j < data.size(); j++) {
                scalar_t dataLoss = mLossFunction->processLoss(model, data[j]);
                if (dataLoss > mThreshold) {
                    currentLoss += 1;
                    inliers[j] = false;
                } else {
                    inliers[j] = true;
                }
            }
            return currentLoss;
        }

        const LossFunction<Model, Type> *getLossFunction() {
            return mLossFunction;
        }

    private:
        Estimator<Model, Type> *mEstimator;
        const LossFunction<Model, Type> *mLossFunction;

        scalar_t mThreshold;

    };

    template <typename Model, typename Type> class LMedS : public Ransac<Model, Type> {

    public:
        explicit LMedS(Estimator<Model, Type> *estimator, LossFunction<Model, Type> *lossFunction)
        : Ransac<Model, Type>(estimator, lossFunction, 0) {

        }

    protected:
        scalar_t processLoss(const List<Type> &data, const Model &model, List<bool> &inliers) override {
            inliers.resize(data.size(), true);

            List<scalar_t> loss;
            loss.resize(data.size());

            for (size_t j = 0; j < data.size(); j++) {
                loss[j] = this->getLossFunction()->processLoss(model, data[j]);
            }

            return median(loss);
        }

    };


    template <typename Model, typename Type> class Mlesac : public Ransac<Model, Type> {

    public:
        explicit Mlesac(Estimator<Model, Type> *estimator, LossFunction<Model, Type> *lossFunction, scalar_t threshold)
                : Ransac<Model, Type>(estimator, lossFunction, threshold) {
        }

    protected:
        scalar_t processLoss(const List<Type> &data, const Model &model, List<bool> &inliers) override {
            scalar_t currentLoss = 0;
            inliers.resize(data.size());

            for (size_t j = 0; j < data.size(); j++) {
                scalar_t dataLoss = this->getLossFunction()->processLoss(model, data[j]);
                if (dataLoss > this->getThreshold()) {
                    currentLoss += this->getThreshold();
                    inliers[j] = false;
                } else {
                    currentLoss += dataLoss;
                    inliers[j] = true;
                }
            }
            return currentLoss;
        }

    };


}

#endif