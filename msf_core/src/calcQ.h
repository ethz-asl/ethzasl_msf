/*

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

#ifndef CALCQ_H_
#define CALCQ_H_


#include <Eigen/Eigen>



template <class Derived, class DerivedQ> void calc_Q(
            double dt,
            const Eigen::Quaternion<double> & q,
            const Eigen::MatrixBase<Derived> & ew,
            const Eigen::MatrixBase<Derived> & ea,
            const Eigen::MatrixBase<Derived> & n_a,
            const Eigen::MatrixBase<Derived> & n_ba,
            const Eigen::MatrixBase<Derived> & n_w,
            const Eigen::MatrixBase<Derived> & n_bw,
            double n_L,
            const Eigen::MatrixBase<Derived> & n_qvw,
            const Eigen::MatrixBase<Derived> & n_qci,
            const Eigen::MatrixBase<Derived> & n_pic,
            Eigen::MatrixBase<DerivedQ> &  Qd)
{

	double q1=q.w(), q2=q.x(), q3=q.y(), q4=q.z();
	double ew1=ew(0), ew2=ew(1), ew3=ew(2);
	double ea1=ea(0), ea2=ea(1), ea3=ea(2);
	double n_a1=n_a(0), n_a2=n_a(1), n_a3=n_a(2);
	double n_ba1=n_ba(0), n_ba2=n_ba(1), n_ba3=n_ba(2);
	double n_w1=n_w(0), n_w2=n_w(1), n_w3=n_w(2);
	double n_bw1=n_bw(0), n_bw2=n_bw(1), n_bw3=n_bw(2);
	double n_qvw1=n_qvw(0), n_qvw2=n_qvw(1), n_qvw3=n_qvw(2);

	double t343 = dt*dt;
	double t348 = q1*q4*2.0;
	double t349 = q2*q3*2.0;
	double t344 = t348-t349;
	double t356 = q1*q3*2.0;
	double t357 = q2*q4*2.0;
	double t345 = t356+t357;
	double t350 = q1*q1;
	double t351 = q2*q2;
	double t352 = q3*q3;
	double t353 = q4*q4;
	double t346 = t350+t351-t352-t353;
	double t347 = n_a1*n_a1;
	double t354 = n_a2*n_a2;
	double t355 = n_a3*n_a3;
	double t358 = q1*q2*2.0;
	double t359 = t344*t344;
	double t360 = t345*t345;
	double t361 = t346*t346;
	double t363 = ea2*t345;
	double t364 = ea3*t344;
	double t362 = t363+t364;
	double t365 = t362*t362;
	double t366 = t348+t349;
	double t367 = t350-t351+t352-t353;
	double t368 = q3*q4*2.0;
	double t369 = t356-t357;
	double t370 = t350-t351-t352+t353;
	double t371 = n_w3*n_w3;
	double t372 = t358+t368;
	double t373 = n_w2*n_w2;
	double t374 = n_w1*n_w1;
	double t375 = dt*t343*t346*t347*t366*(1.0/3.0);
	double t376 = t358-t368;
	double t377 = t343*t346*t347*t366*(1.0/2.0);
	double t378 = t366*t366;
	double t379 = t376*t376;
	double t380 = ea1*t367;
	double t391 = ea2*t366;
	double t381 = t380-t391;
	double t382 = ea3*t367;
	double t383 = ea2*t376;
	double t384 = t382+t383;
	double t385 = t367*t367;
	double t386 = ea1*t376;
	double t387 = ea3*t366;
	double t388 = t386+t387;
	double t389 = ea2*t370;
	double t407 = ea3*t372;
	double t390 = t389-t407;
	double t392 = ea1*t372;
	double t393 = ea2*t369;
	double t394 = t392+t393;
	double t395 = ea1*t370;
	double t396 = ea3*t369;
	double t397 = t395+t396;
	double t398 = n_ba1*n_ba1;
	double t399 = n_ba2*n_ba2;
	double t400 = n_ba3*n_ba3;
	double t401 = dt*t343*t345*t355*t370*(1.0/3.0);
	double t402 = t401-dt*t343*t346*t347*t369*(1.0/3.0)-dt*t343*t344*t354*t372*(1.0/3.0);
	double t403 = dt*t343*t354*t367*t372*(1.0/3.0);
	double t404 = t403-dt*t343*t347*t366*t369*(1.0/3.0)-dt*t343*t355*t370*t376*(1.0/3.0);
	double t405 = t343*t345*t355*t370*(1.0/2.0);
	double t406 = dt*t343*t362*t373*t397*(1.0/6.0);
	double t421 = t343*t346*t347*t369*(1.0/2.0);
	double t422 = dt*t343*t362*t371*t394*(1.0/6.0);
	double t423 = t343*t344*t354*t372*(1.0/2.0);
	double t424 = dt*t343*t362*t374*t390*(1.0/6.0);
	double t408 = t405+t406-t421-t422-t423-t424;
	double t409 = t343*t354*t367*t372*(1.0/2.0);
	double t410 = dt*t343*t374*t384*t390*(1.0/6.0);
	double t411 = dt*t343*t373*t388*t397*(1.0/6.0);
	double t463 = t343*t355*t370*t376*(1.0/2.0);
	double t464 = t343*t347*t366*t369*(1.0/2.0);
	double t465 = dt*t343*t371*t381*t394*(1.0/6.0);
	double t412 = t409+t410+t411-t463-t464-t465;
	double t413 = t369*t369;
	double t414 = t372*t372;
	double t415 = t370*t370;
	double t416 = t343*t354*t359*(1.0/2.0);
	double t417 = t343*t355*t360*(1.0/2.0);
	double t418 = t343*t347*t361*(1.0/2.0);
	double t419 = t416+t417+t418-dt*t343*t365*t371*(1.0/6.0)-dt*t343*t365*t373*(1.0/6.0)-dt*t343*t365*t374*(1.0/6.0);
	double t453 = t343*t344*t354*t367*(1.0/2.0);
	double t454 = t343*t345*t355*t376*(1.0/2.0);
	double t420 = t377-t453-t454;
	double t426 = ew2*t362;
	double t427 = ew3*t362;
	double t425 = t426-t427;
	double t428 = dt*t365;
	double t429 = ew1*ew1;
	double t430 = ew2*ew2;
	double t431 = ew3*ew3;
	double t432 = t430+t431;
	double t433 = t362*t432;
	double t434 = ew1*t343*t365;
	double t435 = t429+t431;
	double t436 = t362*t435;
	double t443 = ew2*ew3*t362;
	double t437 = t433+t436-t443;
	double t438 = ew1*t362*t394;
	double t511 = ew1*t362*t397;
	double t439 = t438-t511;
	double t440 = t343*t439*(1.0/2.0);
	double t441 = t429+t430;
	double t442 = t362*t441;
	double t444 = t390*t432;
	double t445 = ew2*t394;
	double t446 = ew3*t397;
	double t447 = t445+t446;
	double t448 = ew1*ew2*t362;
	double t449 = ew1*ew3*t362;
	double t450 = ew1*ew3*t362*(1.0/2.0);
	double t451 = dt*t362;
	double t452 = ew1*t343*t362*(1.0/2.0);
	double t455 = dt*t343*t362*t374*t384*(1.0/6.0);
	double t456 = t343*t347*t378*(1.0/2.0);
	double t457 = t343*t355*t379*(1.0/2.0);
	double t458 = t381*t381;
	double t459 = t384*t384;
	double t460 = t343*t354*t385*(1.0/2.0);
	double t461 = t388*t388;
	double t462 = t456+t457+t460-dt*t343*t371*t458*(1.0/6.0)-dt*t343*t374*t459*(1.0/6.0)-dt*t343*t373*t461*(1.0/6.0);
	double t466 = t433+t442-t443;
	double t467 = ew1*t362*t388;
	double t468 = ew1*t362*t381;
	double t469 = t467+t468;
	double t470 = t343*t469*(1.0/2.0);
	double t471 = t384*t432;
	double t472 = ew2*t381;
	double t479 = ew3*t388;
	double t473 = t472-t479;
	double t474 = -t433+t448+t449;
	double t475 = dt*t343*t346*t366*t398*(1.0/3.0);
	double t476 = dt*t346*t347*t366;
	double t477 = ew2*ew3*t381;
	double t492 = t388*t435;
	double t478 = t471+t477-t492;
	double t480 = t472-t479;
	double t481 = ew1*ew3*t381;
	double t482 = ew1*ew2*t388;
	double t483 = t471+t481+t482;
	double t484 = ew2*ew3*t388;
	double t486 = t381*t441;
	double t485 = t471+t484-t486;
	double t487 = t394*t441;
	double t488 = ew2*ew3*t397;
	double t489 = t444+t487+t488;
	double t490 = t397*t435;
	double t491 = ew2*ew3*t394;
	double t493 = ew1*t381*t397;
	double t541 = ew1*t388*t394;
	double t494 = t493-t541;
	double t495 = t343*t494*(1.0/2.0);
	double t496 = ew1*ew2*t397;
	double t527 = ew1*ew3*t394;
	double t497 = t444+t496-t527;
	double t498 = ew2*ew3*t381*(1.0/2.0);
	double t499 = ew1*t343*t381*(1.0/2.0);
	double t500 = t384*t432*(1.0/2.0);
	double t501 = ew2*ew3*t388*(1.0/2.0);
	double t502 = n_bw1*n_bw1;
	double t503 = n_bw3*n_bw3;
	double t504 = t343*t347*t413*(1.0/2.0);
	double t505 = t343*t354*t414*(1.0/2.0);
	double t506 = t397*t397;
	double t507 = t390*t390;
	double t508 = t343*t355*t415*(1.0/2.0);
	double t509 = t394*t394;
	double t510 = t504+t505+t508-dt*t343*t373*t506*(1.0/6.0)-dt*t343*t371*t509*(1.0/6.0)-dt*t343*t374*t507*(1.0/6.0);
	double t512 = -t444+t490+t491;
	double t513 = t397*t437*(1.0/2.0);
	double t514 = t362*t394*t429;
	double t515 = dt*t362*t397;
	double t516 = t362*t489*(1.0/2.0);
	double t517 = t394*t466*(1.0/2.0);
	double t518 = t362*t397*t429;
	double t519 = t516+t517+t518;
	double t520 = dt*t362*t394;
	double t521 = t440+t520-dt*t343*t519*(1.0/3.0);
	double t522 = t371*t521;
	double t523 = t362*t447;
	double t524 = t390*t425;
	double t525 = t523+t524;
	double t526 = t343*t525*(1.0/2.0);
	double t528 = t425*t447;
	double t529 = t390*t474*(1.0/2.0);
	double t530 = t528+t529-t362*t497*(1.0/2.0);
	double t531 = dt*t343*t530*(1.0/3.0);
	double t532 = dt*t362*t390;
	double t533 = t526+t531+t532;
	double t534 = t374*t533;
	double t535 = dt*t343*t345*t370*t400*(1.0/3.0);
	double t536 = dt*t345*t355*t370;
	double t537 = t381*t489*(1.0/2.0);
	double t538 = t388*t397*t429;
	double t539 = t537+t538-t394*t485*(1.0/2.0);
	double t540 = dt*t343*t539*(1.0/3.0);
	double t542 = t495+t540-dt*t381*t394;
	double t543 = t388*t512*(1.0/2.0);
	double t544 = t381*t394*t429;
	double t545 = t543+t544-t397*t478*(1.0/2.0);
	double t546 = dt*t343*t545*(1.0/3.0);
	double t547 = t495+t546-dt*t388*t397;
	double t548 = t373*t547;
	double t549 = t384*t447;
	double t550 = t549-t390*t473;
	double t551 = t343*t550*(1.0/2.0);
	double t552 = t384*t497*(1.0/2.0);
	double t553 = t390*t483*(1.0/2.0);
	double t554 = t447*t473;
	double t555 = t552+t553+t554;
	double t556 = dt*t384*t390;
	double t557 = t551+t556-dt*t343*t555*(1.0/3.0);
	double t558 = dt*t343*t367*t372*t399*(1.0/3.0);
	double t559 = dt*t354*t367*t372;
	double t560 = t548+t558+t559-t371*t542-t374*t557-dt*t347*t366*t369-dt*t355*t370*t376-dt*t343*t366*t369*t398*(1.0/3.0)-dt*t343*t370*t376*t400*(1.0/3.0);
	double t561 = ew1*t343*t394*t397;
	double t562 = ew1*t343*t397*(1.0/2.0);
	double t563 = n_bw2*n_bw2;
	double t564 = dt*t343*t362*t374*(1.0/6.0);
	double t565 = dt*t343*t374*t390*(1.0/6.0);
	double t566 = ew1*ew2*t362*(1.0/2.0);
	double t567 = -t433+t450+t566;
	double t568 = dt*t343*t567*(1.0/3.0);
	double t569 = t343*t425*(1.0/2.0);
	double t570 = t451+t568+t569;
	double t571 = dt*t343*t362*t373*t432*(1.0/6.0);
	double t572 = dt*t343*t362*t371*t432*(1.0/6.0);
	double t573 = t571+t572-t374*t570;
	double t574 = ew1*ew2*t397*(1.0/2.0);
	double t575 = t444+t574-ew1*ew3*t394*(1.0/2.0);
	double t576 = t343*t447*(1.0/2.0);
	double t577 = dt*t390;
	double t578 = t576+t577-dt*t343*t575*(1.0/3.0);
	double t579 = dt*t343*t371*t394*t432*(1.0/6.0);
	double t580 = t579-t374*t578-dt*t343*t373*t397*t432*(1.0/6.0);
	double t581 = dt*t343*t373*t388*(1.0/6.0);
	double t582 = t362*t432*(1.0/2.0);
	double t583 = ew2*ew3*t362*(1.0/2.0);
	double t584 = t362*t429;
	double t585 = t583+t584;
	double t586 = ew3*t473;
	double t587 = ew1*ew2*t384*(1.0/2.0);
	double t588 = t586+t587;
	double t589 = dt*t343*t588*(1.0/3.0);
	double t590 = t374*(t589-ew3*t343*t384*(1.0/2.0));
	double t591 = t388*t429;
	double t592 = t498+t591;
	double t593 = dt*t343*t592*(1.0/3.0);
	double t594 = t499+t593;
	double t595 = -t492+t498+t500;
	double t596 = dt*t343*t595*(1.0/3.0);
	double t597 = dt*t388;
	double t598 = -t499+t596+t597;
	double t599 = t590-t371*t594-t373*t598;
	double t600 = t397*t429;
	double t601 = ew2*ew3*t394*(1.0/2.0);
	double t602 = ew1*t343*t394*(1.0/2.0);
	double t603 = ew3*t447;
	double t604 = t603-ew1*ew2*t390*(1.0/2.0);
	double t605 = dt*t343*t604*(1.0/3.0);
	double t606 = ew3*t343*t390*(1.0/2.0);
	double t607 = t605+t606;
	double t608 = t374*t607;
	double t609 = t390*t432*(1.0/2.0);
	double t610 = dt*t397;
	double t611 = t430*(1.0/2.0);
	double t612 = t431*(1.0/2.0);
	double t613 = t611+t612;
	double t614 = ew1*t343*(1.0/2.0);
	double t615 = dt*t343*t362*t371*(1.0/6.0);
	double t616 = dt*t343*t371*t381*(1.0/6.0);
	double t617 = dt*t343*t371*t394*(1.0/6.0);
	double t618 = ew2*t425;
	double t619 = t450+t618;
	double t620 = dt*t343*t619*(1.0/3.0);
	double t621 = ew2*t343*t362*(1.0/2.0);
	double t622 = t620+t621;
	double t623 = dt*t343*t585*(1.0/3.0);
	double t624 = t381*t429;
	double t625 = t501+t624;
	double t626 = dt*t343*t625*(1.0/3.0);
	double t627 = ew1*t343*t388*(1.0/2.0);
	double t628 = ew2*t473;
	double t629 = t628-ew1*ew3*t384*(1.0/2.0);
	double t630 = dt*t343*t629*(1.0/3.0);
	double t631 = t630-ew2*t343*t384*(1.0/2.0);
	double t632 = -t486+t500+t501;
	double t633 = dt*t343*t632*(1.0/3.0);
	double t634 = dt*t381;
	double t635 = t627+t633+t634;
	double t636 = ew2*t447;
	double t637 = ew1*ew3*t390*(1.0/2.0);
	double t638 = t636+t637;
	double t639 = dt*t343*t638*(1.0/3.0);
	double t640 = ew2*t343*t390*(1.0/2.0);
	double t641 = t639+t640;
	double t642 = t394*t429;
	double t643 = ew2*ew3*t397*(1.0/2.0);
	double t644 = t487+t609+t643;
	double t645 = dt*t343*t644*(1.0/3.0);
	double t646 = t562+t645-dt*t394;
	double t647 = t371*t646;
	double t648 = ew2*t343*(1.0/2.0);
	double t649 = dt*ew1*ew3*t343*(1.0/6.0);
	double t650 = t648+t649;
	double t651 = t374*t650;
	double t652 = t651-dt*t343*t371*t613*(1.0/3.0);
	double t653 = dt*ew2*ew3*t343*(1.0/6.0);
	double t654 = t614+t653;
	double t655 = t371*t654;
	double t656 = dt*t343*t397*t563*(1.0/6.0);
	double t657 = dt*ew1*t343*t563*(1.0/6.0);
	double t658 = dt*t343*t369*t398*(1.0/6.0);
	double t659 = t343*t369*t398*(1.0/2.0);
	double t660 = dt*t343*t344*t399*(1.0/6.0);
	double t661 = t343*t344*t399*(1.0/2.0);
	double t662 = dt*t343*t376*t400*(1.0/6.0);
	double t663 = t343*t376*t400*(1.0/2.0);
	Qd(0,0) = dt*t343*t347*t361*(1.0/3.0)+dt*t343*t354*t359*(1.0/3.0)+dt*t343*t355*t360*(1.0/3.0);
	Qd(0,1) = t375-dt*t343*t345*t355*(t358-q3*q4*2.0)*(1.0/3.0)-dt*t343*t344*t354*t367*(1.0/3.0);
	Qd(0,2) = t402;
	Qd(0,3) = t419;
	Qd(0,4) = t420;
	Qd(0,5) = t408;
	Qd(0,6) = t564;
	Qd(0,8) = t615;
	Qd(0,12) = dt*t343*t346*t398*(-1.0/6.0);
	Qd(0,13) = t660;
	Qd(0,14) = dt*t343*t345*t400*(-1.0/6.0);
	Qd(1,0) = t375-dt*t343*t344*t354*t367*(1.0/3.0)-dt*t343*t345*t355*t376*(1.0/3.0);
	Qd(1,1) = dt*t343*t347*t378*(1.0/3.0)+dt*t343*t355*t379*(1.0/3.0)+dt*t343*t354*t385*(1.0/3.0);
	Qd(1,2) = t404;
	Qd(1,3) = t377+t455-t343*t344*t354*t367*(1.0/2.0)-t343*t345*t355*t376*(1.0/2.0)-dt*t343*t362*t371*t381*(1.0/6.0)-dt*t343*t362*t373*t388*(1.0/6.0);
	Qd(1,4) = t462;
	Qd(1,5) = t412;
	Qd(1,6) = dt*t343*t374*t384*(-1.0/6.0);
	Qd(1,7) = t581;
	Qd(1,8) = t616;
	Qd(1,12) = dt*t343*t366*t398*(-1.0/6.0);
	Qd(1,13) = dt*t343*t367*t399*(-1.0/6.0);
	Qd(1,14) = t662;
	Qd(2,0) = t402;
	Qd(2,1) = t404;
	Qd(2,2) = dt*t343*t347*t413*(1.0/3.0)+dt*t343*t354*t414*(1.0/3.0)+dt*t343*t355*t415*(1.0/3.0);
	Qd(2,3) = t408;
	Qd(2,4) = t412;
	Qd(2,5) = t510;
	Qd(2,6) = t565;
	Qd(2,7) = dt*t343*t373*t397*(-1.0/6.0);
	Qd(2,8) = t617;
	Qd(2,12) = t658;
	Qd(2,13) = dt*t343*t372*t399*(-1.0/6.0);
	Qd(2,14) = dt*t343*t370*t400*(-1.0/6.0);
	Qd(3,0) = t419;
	Qd(3,1) = t420;
	Qd(3,2) = t408;
	Qd(3,3) = t374*(t428+t343*t362*t425+dt*t343*(t362*(t448+t449-t362*t432)+t425*t425)*(1.0/3.0))+t373*(t428-t434+dt*t343*(t365*t429-t362*t437)*(1.0/3.0))+t371*(t428+t434+dt*t343*(t365*t429-t362*(t433+t442-ew2*ew3*t362))*(1.0/3.0))+dt*t347*t361+dt*t354*t359+dt*t355*t360+dt*t343*t359*t399*(1.0/3.0)+dt*t343*t361*t398*(1.0/3.0)+dt*t343*t360*t400*(1.0/3.0);
	Qd(3,4) = t475+t476-dt*t344*t354*t367-dt*t345*t355*t376-dt*t343*t344*t367*t399*(1.0/3.0)-dt*t343*t345*t376*t400*(1.0/3.0);
	Qd(3,5) = t522+t534+t535+t536-t373*(t440+t515-dt*t343*(t513+t514+t362*(t490+t491-t390*t432)*(1.0/2.0))*(1.0/3.0))-dt*t346*t347*t369-dt*t344*t354*t372-dt*t343*t346*t369*t398*(1.0/3.0)-dt*t343*t344*t372*t399*(1.0/3.0);
	Qd(3,6) = t573;
	Qd(3,8) = -t371*(t451+t452-dt*t343*(t442+t582-ew2*ew3*t362*(1.0/2.0))*(1.0/3.0))-t374*t622+t373*(t452-dt*t343*t585*(1.0/3.0));
	Qd(3,9) = dt*t343*t362*t502*(-1.0/6.0);
	Qd(3,11) = dt*t343*t362*t503*(-1.0/6.0);
	Qd(3,12) = t343*t346*t398*(-1.0/2.0);
	Qd(3,13) = t661;
	Qd(3,14) = t343*t345*t400*(-1.0/2.0);
	Qd(4,0) = t377-t453-t454+t455-dt*t343*t362*t371*t381*(1.0/6.0)-dt*t343*t362*t373*t388*(1.0/6.0);
	Qd(4,1) = t462;
	Qd(4,2) = t412;
	Qd(4,3) = t475+t476-t374*(t343*(t384*t425-t362*t473)*(1.0/2.0)-dt*t343*(t362*t483*(1.0/2.0)-t384*t474*(1.0/2.0)+t425*t473)*(1.0/3.0)+dt*t362*t384)+t371*(t470+dt*t362*t381+dt*t343*(t362*t485*(1.0/2.0)-t381*t466*(1.0/2.0)+t362*t388*t429)*(1.0/3.0))+t373*(-t470+dt*t362*t388+dt*t343*(t388*t437*(-1.0/2.0)+t362*t478*(1.0/2.0)+t362*t381*t429)*(1.0/3.0))-dt*t344*t354*t367-dt*t345*t355*t376-dt*t343*t344*t367*t399*(1.0/3.0)-dt*t343*t345*t376*t400*(1.0/3.0);
	Qd(4,4) = -t374*(-dt*t459+dt*t343*(t384*t483-t480*t480)*(1.0/3.0)+t343*t384*t473)+t373*(dt*t461+dt*t343*(t388*t478+t429*t458)*(1.0/3.0)-ew1*t343*t381*t388)+t371*(dt*t458+dt*t343*(t381*t485+t429*t461)*(1.0/3.0)+ew1*t343*t381*t388)+dt*t347*t378+dt*t355*t379+dt*t354*t385+dt*t343*t378*t398*(1.0/3.0)+dt*t343*t385*t399*(1.0/3.0)+dt*t343*t379*t400*(1.0/3.0);
	Qd(4,5) = t560;
	Qd(4,6) = -t374*(-dt*t384+t343*t473*(1.0/2.0)+dt*t343*(t471+ew1*ew2*t388*(1.0/2.0)+ew1*ew3*t381*(1.0/2.0))*(1.0/3.0))+dt*t343*t371*t381*t432*(1.0/6.0)+dt*t343*t373*t388*t432*(1.0/6.0);
	Qd(4,7) = t599;
	Qd(4,8) = -t374*t631-t371*t635-t373*(t626-ew1*t343*t388*(1.0/2.0));
	Qd(4,9) = dt*t343*t384*t502*(1.0/6.0);
	Qd(4,10) = dt*t343*t388*t563*(-1.0/6.0);
	Qd(4,11) = dt*t343*t381*t503*(-1.0/6.0);
	Qd(4,12) = t343*t366*t398*(-1.0/2.0);
	Qd(4,13) = t343*t367*t399*(-1.0/2.0);
	Qd(4,14) = t663;
	Qd(5,0) = t408;
	Qd(5,1) = t412;
	Qd(5,2) = t510;
	Qd(5,3) = t522+t534+t535+t536-t373*(t440+t515-dt*t343*(t513+t514+t362*t512*(1.0/2.0))*(1.0/3.0))-dt*t346*t347*t369-dt*t344*t354*t372-dt*t343*t346*t369*t398*(1.0/3.0)-dt*t343*t344*t372*t399*(1.0/3.0);
	Qd(5,4) = t560;
	Qd(5,5) = -t371*(t561-dt*t509+dt*t343*(t394*t489-t429*t506)*(1.0/3.0))+t373*(t561+dt*t506-dt*t343*(t397*t512-t429*t509)*(1.0/3.0))+t374*(dt*t507-dt*t343*(t390*t497-t447*t447)*(1.0/3.0)+t343*t390*t447)+dt*t347*t413+dt*t354*t414+dt*t355*t415+dt*t343*t398*t413*(1.0/3.0)+dt*t343*t399*t414*(1.0/3.0)+dt*t343*t400*t415*(1.0/3.0);
	Qd(5,6) = t580;
	Qd(5,7) = t608+t371*(dt*t343*(t600-ew2*ew3*t394*(1.0/2.0))*(1.0/3.0)-ew1*t343*t394*(1.0/2.0))+t373*(t602+t610-dt*t343*(t490+t601-t390*t432*(1.0/2.0))*(1.0/3.0));
	Qd(5,8) = t647-t374*t641-t373*(t562+dt*t343*(t642-ew2*ew3*t397*(1.0/2.0))*(1.0/3.0));
	Qd(5,9) = dt*t343*t390*t502*(-1.0/6.0);
	Qd(5,10) = t656;
	Qd(5,11) = dt*t343*t394*t503*(-1.0/6.0);
	Qd(5,12) = t659;
	Qd(5,13) = t343*t372*t399*(-1.0/2.0);
	Qd(5,14) = t343*t370*t400*(-1.0/2.0);
	Qd(6,0) = t564;
	Qd(6,2) = t565;
	Qd(6,3) = t573;
	Qd(6,5) = t580;
	Qd(6,6) = t374*(dt-dt*t343*t432*(1.0/3.0))+dt*t343*t502*(1.0/3.0);
	Qd(6,8) = t652;
	Qd(6,9) = t343*t502*(-1.0/2.0);
	Qd(7,0) = dt*t343*t362*t373*(1.0/6.0);
	Qd(7,1) = t581;
	Qd(7,2) = dt*t343*t373*t397*(-1.0/6.0);
	Qd(7,3) = -t371*(t452+t623)-t374*(dt*t343*(t566-ew3*t425)*(1.0/3.0)-ew3*t343*t362*(1.0/2.0))+t373*(-t451+t452+dt*t343*(t436+t582-t583)*(1.0/3.0));
	Qd(7,4) = t599;
	Qd(7,5) = t608+t373*(t602+t610-dt*t343*(t490+t601-t609)*(1.0/3.0))-t371*(t602-dt*t343*(t600-t601)*(1.0/3.0));
	Qd(7,6) = -t374*(ew3*t343*(1.0/2.0)-dt*ew1*ew2*t343*(1.0/6.0))-dt*t343*t373*t613*(1.0/3.0);
	Qd(7,7) = t373*(dt-dt*t343*t435*(1.0/3.0))+dt*t343*t563*(1.0/3.0)+dt*t343*t371*t429*(1.0/3.0)+dt*t343*t374*t431*(1.0/3.0);
	Qd(7,8) = t655-t373*(t614-dt*ew2*ew3*t343*(1.0/6.0))-dt*ew2*ew3*t343*t374*(1.0/3.0);
	Qd(7,9) = dt*ew3*t343*t502*(1.0/6.0);
	Qd(7,10) = t343*t563*(-1.0/2.0);
	Qd(7,11) = dt*ew1*t343*t503*(-1.0/6.0);
	Qd(8,0) = t615;
	Qd(8,1) = t616;
	Qd(8,2) = t617;
	Qd(8,3) = -t374*t622-t371*(t451+t452-dt*t343*(t442+t582-t583)*(1.0/3.0))+t373*(t452-t623);
	Qd(8,4) = -t374*t631-t371*t635-t373*(t626-t627);
	Qd(8,5) = t647-t374*t641-t373*(t562+dt*t343*(t642-t643)*(1.0/3.0));
	Qd(8,6) = t652;
	Qd(8,7) = t655-t373*(t614-t653)-dt*ew2*ew3*t343*t374*(1.0/3.0);
	Qd(8,8) = t371*(dt-dt*t343*t441*(1.0/3.0))+dt*t343*t503*(1.0/3.0)+dt*t343*t373*t429*(1.0/3.0)+dt*t343*t374*t430*(1.0/3.0);
	Qd(8,9) = dt*ew2*t343*t502*(-1.0/6.0);
	Qd(8,10) = t657;
	Qd(8,11) = t343*t503*(-1.0/2.0);
	Qd(9,3) = dt*t343*t362*t502*(-1.0/6.0);
	Qd(9,5) = dt*t343*t390*t502*(-1.0/6.0);
	Qd(9,6) = t343*t502*(-1.0/2.0);
	Qd(9,8) = dt*ew2*t343*t502*(-1.0/6.0);
	Qd(9,9) = dt*t502;
	Qd(10,3) = dt*t343*t362*t563*(-1.0/6.0);
	Qd(10,4) = dt*t343*t388*t563*(-1.0/6.0);
	Qd(10,5) = t656;
	Qd(10,7) = t343*t563*(-1.0/2.0);
	Qd(10,8) = t657;
	Qd(10,10) = dt*t563;
	Qd(11,3) = dt*t343*t362*t503*(-1.0/6.0);
	Qd(11,4) = dt*t343*t381*t503*(-1.0/6.0);
	Qd(11,5) = dt*t343*t394*t503*(-1.0/6.0);
	Qd(11,7) = dt*ew1*t343*t503*(-1.0/6.0);
	Qd(11,8) = t343*t503*(-1.0/2.0);
	Qd(11,11) = dt*t503;
	Qd(12,0) = dt*t343*t346*t398*(-1.0/6.0);
	Qd(12,1) = dt*t343*t366*t398*(-1.0/6.0);
	Qd(12,2) = t658;
	Qd(12,3) = t343*t346*t398*(-1.0/2.0);
	Qd(12,4) = t343*t366*t398*(-1.0/2.0);
	Qd(12,5) = t659;
	Qd(12,12) = dt*t398;
	Qd(13,0) = t660;
	Qd(13,1) = dt*t343*t367*t399*(-1.0/6.0);
	Qd(13,2) = dt*t343*t372*t399*(-1.0/6.0);
	Qd(13,3) = t661;
	Qd(13,4) = t343*t367*t399*(-1.0/2.0);
	Qd(13,5) = t343*t372*t399*(-1.0/2.0);
	Qd(13,13) = dt*t399;
	Qd(14,0) = dt*t343*t345*t400*(-1.0/6.0);
	Qd(14,1) = t662;
	Qd(14,2) = dt*t343*t370*t400*(-1.0/6.0);
	Qd(14,3) = t343*t345*t400*(-1.0/2.0);
	Qd(14,4) = t663;
	Qd(14,5) = t343*t370*t400*(-1.0/2.0);
	Qd(14,14) = dt*t400;
	Qd(15,15) = dt*(n_L*n_L);
	Qd(16,16) = dt*(n_qvw1*n_qvw1);
	Qd(17,17) = dt*(n_qvw2*n_qvw2);
	Qd(18,18) = dt*(n_qvw3*n_qvw3);
	Qd(19,19) = dt*(n_qci[0]*n_qci[0]);
	Qd(20,20) = dt*(n_qci[1]*n_qci[1]);
	Qd(21,21) = dt*(n_qci[2]*n_qci[2]);
	Qd(22,22) = dt*(n_pic[0]*n_pic[0]);
	Qd(23,23) = dt*(n_pic[1]*n_pic[1]);
	Qd(24,24) = dt*(n_pic[2]*n_pic[2]);
};

#endif /* CALCQ_H_ */
