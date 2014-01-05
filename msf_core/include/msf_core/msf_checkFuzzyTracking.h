/*
 * Copyright (C) 2011-2012 Stephan Weiss, ASL, ETH Zurich, Switzerland
 * You can contact the author at <stephan dot weiss at ieee dot org>
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef MSF_CHECKFUZZYTRACKING_H_
#define MSF_CHECKFUZZYTRACKING_H_

#include <msf_core/msf_tools.h>

namespace msf_core {
template<typename EKFState_T, typename NONTEMPORALDRIFTINGTYPE>
class CheckFuzzyTracking {
 private:
  enum {
    qbuffRowsAtCompiletime = msf_tmp::StateLengthForType<
        const typename msf_tmp::StripConstReference<NONTEMPORALDRIFTINGTYPE>::result_t&>::value,
    nBuff_ = 30  ///< Buffer size for median non drifting state values.
  };

  int nontemporaldrifting_inittimer_;  ///< A counter for fuzzy tracking detection
  // If there is no non temporal drifting state this matrix will have zero rows,
  // to make use of it illegal.
  Eigen::Matrix<double, nBuff_, qbuffRowsAtCompiletime> qbuff_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void Reset() {
    // Buffer for vision failure check.
    nontemporaldrifting_inittimer_ = 1;
    qbuff_ = Eigen::Matrix<double, nBuff_, qbuffRowsAtCompiletime>::Constant(0);
  }

  CheckFuzzyTracking() : nontemporaldrifting_inittimer_(0) {
    Reset();
  }

  bool Check(shared_ptr<EKFState_T> delaystate, EKFState_T& buffstate,
             double fuzzythres) {
    // For now make sure the non drifting state is a quaternion.
    bool isfuzzy = false;

    const bool isquaternion =
        msf_tmp::IsQuaternionType<
            typename msf_tmp::StripConstReference<NONTEMPORALDRIFTINGTYPE>::result_t>::value;
    static_assert(
        isquaternion,
        "Assumed that the non drifting state is a Quaternion, "
        "which is not the case for the currently defined state vector. If you "
        "want to use an euclidean state, please first adapt qbuff and the error "
        "detection routines");
    typedef typename EKFState_T::StateSequence_T StateSequence_T;

    enum {
      indexOfStateWithoutTemporalDrift = msf_tmp::IndexOfBestNonTemporalDriftingState<
          StateSequence_T>::value,
      /// Error state length.
      nErrorStatesAtCompileTime = EKFState_T::nErrorStatesAtCompileTime,
      /// Complete state length.
      nStatesAtCompileTime = EKFState_T::nStatesAtCompileTime
    };

    // Update qbuff_ and check for fuzzy tracking.
    if (nontemporaldrifting_inittimer_ > nBuff_) {
      // should be unit quaternion if no error
      Eigen::Quaternion<double> errq =
          const_cast<const EKFState_T&>(*delaystate)
              .template Get<indexOfStateWithoutTemporalDrift>().conjugate() *
      Eigen::Quaternion<double>(
          GetMedian(qbuff_.template block<nBuff_, 1> (0, 3)),
          GetMedian(qbuff_.template block<nBuff_, 1> (0, 0)),
          GetMedian(qbuff_.template block<nBuff_, 1> (0, 1)),
          GetMedian(qbuff_.template block<nBuff_, 1> (0, 2))
      );

      if (std::max(errq.vec().maxCoeff(), -errq.vec().minCoeff()) /
          fabs(errq.w()) * 2 > fuzzythres) {  // Fuzzy tracking (small angle approx).
        MSF_WARN_STREAM("Fuzzy tracking triggered: " <<
            std::max(errq.vec().maxCoeff(), -errq.vec().minCoeff()) /
            fabs(errq.w())*2 << " limit: " << fuzzythres <<"\n");

        // Copy the non propagation states back from the buffer.
        boost::fusion::for_each(
          delaystate->statevars,
          msf_tmp::CopyNonPropagationStates<EKFState_T>(buffstate)
        );
        static_assert(static_cast<int>(
            EKFState_T::nPropagatedCoreErrorStatesAtCompileTime) == 9,
                "Assumed that nPropagatedCoreStates == 9, "
                "which is not the case");
        isfuzzy = true;
      } else {  // If tracking ok: Update mean and 3sigma of past N non drifting state values.
        qbuff_.template block<1, 4>(nontemporaldrifting_inittimer_ - nBuff_ - 1, 0) =
            Eigen::Matrix<double, 1, 4>(
                const_cast<const EKFState_T&>(*delaystate).
                template Get<indexOfStateWithoutTemporalDrift>().coeffs());
         nontemporaldrifting_inittimer_ = (nontemporaldrifting_inittimer_) % nBuff_ + nBuff_ + 1;
       }
     } else {  // At beginning get mean and 3sigma of past N non drifting state values.
       qbuff_. template block<1, 4> (nontemporaldrifting_inittimer_ - 1, 0) =
           Eigen::Matrix<double, 1, 4>(
               const_cast<const EKFState_T&>(*delaystate).
               template Get<indexOfStateWithoutTemporalDrift>().coeffs());
       nontemporaldrifting_inittimer_++;
     }
     return isfuzzy;
  }
};

template<typename EKFState_T>
class CheckFuzzyTracking<EKFState_T, mpl_::void_> {
 public:
  bool Check(shared_ptr<EKFState_T> UNUSEDPARAM(delaystate),
             EKFState_T& UNUSEDPARAM(buffstate),
             double UNUSEDPARAM(fuzzythres)) {
    return false;
  }
  void Reset() { }
};
}  // namespace msf_core
#endif  // MSF_CHECKFUZZYTRACKING_H_
