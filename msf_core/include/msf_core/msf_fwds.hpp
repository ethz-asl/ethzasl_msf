/*
 *  Created on: Nov 7, 2012
 *      Author: slynen
 */


#ifndef MSF_FWD_HPP_
#define MSF_FWD_HPP_

namespace msf_core{
//forwards
//state variable
template<typename type_T, int name_T, bool PROPAGATED = false>
struct StateVar_T;

//the state
template<typename stateVector_T>
struct GenericState_T;


}
#endif /* MSF_FWD_HPP_ */
