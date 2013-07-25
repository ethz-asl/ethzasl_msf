/*

 Copyright (c) 2013, Simon Lynen, ASL, ETH Zurich, Switzerland
 You can contact the author at <slynen at ethz dot ch>
 Copyright (c) 2010, Stephan Weiss, ASL, ETH Zurich, Switzerland
 You can contact the author at <stephan dot weiss at ieee dot org>

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of ETHZ-ASL nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
        const typename
        msf_tmp::StripConstReference<NONTEMPORALDRIFTINGTYPE>::result_t&>::value,
    nBuff_ = 30  ///< Buffer size for median non drifting state values.
  };

  int nontemporaldrifting_inittimer_;  ///< A counter for fuzzy tracking detection
  // If there is no non temporal drifting state this matrix will have zero rows,
  // to make use of it illegal.
  Eigen::Matrix<double, nBuff_, qbuffRowsAtCompiletime> qbuff_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ;

  void reset() {
    // Buffer for vision failure check.
    nontemporaldrifting_inittimer_ = 1;
    qbuff_ = Eigen::Matrix<double, nBuff_, qbuffRowsAtCompiletime>::Constant(0);
  }

  CheckFuzzyTracking() {
    reset();
  }

  bool check(shared_ptr<EKFState_T> delaystate, EKFState_T& buffstate,
             double fuzzythres) {
    // For now make sure the non drifting state is a quaternion.
    bool isfuzzy = false;

    const bool isquaternion =
        msf_tmp::isQuaternionType<
            typename msf_tmp::StripConstReference<NONTEMPORALDRIFTINGTYPE>::result_t>::value;
    static_assert(
        isquaternion,
        "Assumed that the non drifting state is a Quaternion, "
        "which is not the case for the currently defined state vector. If you "
        "want to use an euclidean state, please first adapt qbuff and the error "
        "detection routines");
    typedef typename EKFState_T::StateDefinition_T StateDefinition_T;
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
              .template get<indexOfStateWithoutTemporalDrift>().conjugate() *
          Eigen::Quaternion<double>(
              getMedian(qbuff_. template block<nBuff_, 1> (0, 3)),
              getMedian(qbuff_. template block<nBuff_, 1> (0, 0)),
              getMedian(qbuff_. template block<nBuff_, 1> (0, 1)),
              getMedian(qbuff_. template block<nBuff_, 1> (0, 2))
          );

          if (std::max(errq.vec().maxCoeff(), -errq.vec().minCoeff()) /
              fabs(errq.w()) * 2 > fuzzythres) // Fuzzy tracking (small angle approx).
      {
        MSF_WARN_STREAM("Fuzzy tracking triggered: " <<
                        std::max(errq.vec().maxCoeff(), -errq.vec().minCoeff()) /
                        fabs(errq.w())*2 << " limit: " << fuzzythres <<"\n");

        // Copy the non propagation states back from the buffer.
        boost::fusion::for_each(
            delaystate->statevars,
            msf_tmp::copyNonPropagationStates<EKFState_T>(buffstate)
        );

        static_assert(static_cast<int>(
            EKFState_T::nPropagatedCoreErrorStatesAtCompileTime) == 9,
                                "Assumed that nPropagatedCoreStates == 9, "
                                "which is not the case");
        isfuzzy = true;
      }
      else // If tracking ok: update mean and 3sigma of past N non drifting state values.
      {
        qbuff_. template block<1, 4>(nontemporaldrifting_inittimer_ - nBuff_ - 1, 0) =
            Eigen::Matrix<double, 1, 4>(const_cast<const EKFState_T&>(*delaystate).
                                        template get<indexOfStateWithoutTemporalDrift>().coeffs());
        nontemporaldrifting_inittimer_ = (nontemporaldrifting_inittimer_) % nBuff_ + nBuff_ + 1;
      }
    }
      else  // At beginning get mean and 3sigma of past N non drifting state values.
      {
        qbuff_. template block<1, 4> (nontemporaldrifting_inittimer_ - 1, 0) =
            Eigen::Matrix<double, 1, 4>(const_cast<const EKFState_T&>(*delaystate).
                                        template get<indexOfStateWithoutTemporalDrift>().coeffs());
        nontemporaldrifting_inittimer_++;
      }
    return isfuzzy;
  }
};

template<typename EKFState_T>
class CheckFuzzyTracking<EKFState_T, mpl_::void_> {
 public:
  bool check(shared_ptr<EKFState_T> UNUSEDPARAM(delaystate),
             EKFState_T& UNUSEDPARAM(buffstate),
             double UNUSEDPARAM(fuzzythres)) {
    return false;
  }
  void reset() {
  }
  ;
};
}
#endif  // MSF_CHECKFUZZYTRACKING_H_
