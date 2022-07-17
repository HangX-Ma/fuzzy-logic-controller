/**
 * @file Membership.cpp
 * @author mahx(MContour) m-contour@qq.com
 * @brief membership functions class
 * @version 0.1
 * @date 2022-07-15
 * 
 * @copyright Copyright (c) 2022 Fuzzy Limited. All rights reserved.
 * 
 */


#include "Membership.h"


namespace fc {
    Membership::Membership(scalar height) : m_height(height) {}


    Membership::~Membership() {}


    scalar Membership::Rectangle(scalar x, scalar start, scalar end) const {
        if (Operation::isNaN(x)) {
            return fc::nan;
        }
        if (Operation::isGreaterOrEqual(x, start) && Operation::isLessOrEqual(x, end)) {
            return m_height * 1.0;
        }

        return m_height * 0.0; 
    }  // Rectangle membership function has mu(x)=1 in range [s,e].


    scalar Membership::Triangle(scalar x, scalar vertexA, scalar vertexB, scalar vertexC) const {
        if (Operation::isNaN(x)) return fc::nan;

        if (Operation::isLessthan(x, vertexA) || Operation::isGreaterThan(x, vertexC)) {
            return m_height * 0.0;

        } // outside the membership area

        if (Operation::isEqual(x, vertexB)) {
            return m_height * 1.0;
        }

        if (Operation::isLessthan(x, vertexB)) {
            if (vertexA == -fc::inf) {
                return m_height * 1.0;
            }
            return m_height * (x - vertexA) / (vertexB - vertexA);
        }
        if (vertexC == fc::inf) {
            return m_height * 1.0;
        }

        return m_height * (vertexC - x) / (vertexC - vertexB);
    } 


    scalar Membership::Trapezoid(scalar x, scalar vertexA, scalar vertexB, scalar vertexC, scalar vertexD) const {
        if (Operation::isNaN(x)) return fc::nan;

        if (Operation::isLessthan(x, vertexA) || Operation::isGreaterThan(x, vertexD)) {
            return m_height * 0.0;
        } // outside the membership area

        if (Operation::isLessthan(x, vertexB)) {
            if (vertexA == -fc::inf) {
                return m_height * 1.0;
            }
            return m_height * Operation::min(scalar(1.0), (x - vertexA) / (vertexB - vertexA));
        }

        if (Operation::isLessthan(x, vertexC)) {
            return m_height * 1.0;
        }

        if (Operation::isLessthan(x, vertexD)) {
            if (vertexD == fc::inf) {
                return m_height * 1.0;
            }
            return m_height * (vertexD - x) / (vertexD - vertexC);
        }

        if (vertexD == fc::inf) {
            return m_height * 1.0;
        }

        return m_height * 0.0;
    }


    scalar Membership::Gaussian(scalar x, scalar mean, scalar standardDeviation) const {
        if (Operation::isNaN(x)) return fc::nan;

        return m_height * std::exp((-(x - mean) * (x - mean)) / (2.0 * standardDeviation * standardDeviation));
    }


    void Membership::setHeight(scalar height) {
        this->m_height = height;
    }


    scalar Membership::getHeight() const {
        return this->m_height;
    }

}
