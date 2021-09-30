/** @file
Validation of the various tools.
*/
#include "tools/tools.hpp"

//////////////////////////////////////////////////

namespace unitair {
namespace tools {

/// Collection of methods for testing tools
class Tests {
    static Dict<bool (*)()> const tests;

    /////////////////////////

    static bool _sinc() {
        return sinc(EPS/2.0) == 1.0    and
               is_close(sinc(PI), 0.0) and
               is_close(sinc(123), -0.003739052769834075213759);
    }

    static bool _imod() {
        try {imod(1, 0); return false;} catch (std::runtime_error const&) {}
        return imod(55124, 19) == (55124 % 19) and
               imod(-443, 5112) == (5112 - 443) and
               imod(-1, 122) == 121 and
               imod(-122, 122) == 0 and
               imod(-123, 122) == 121 and
               imod(0, 342) == 0;
    }

    static bool _clip() {
        return clip(-4, -102, 3) == -4     and
               clip(1.1, -1.0, 0.0) == 0.0 and
               clip(3.0, 3.0, 3.0) == 3.0  and
               clip(1, 3, 9) == 3;
    }

    static bool _is_close() {
        return is_close(PI, 3.14159, 1e-5) and
               not is_close(PI, 3.14159, 1e-6);
    }

    static bool _all_close() {
        RealVecN u(3); u << 1.0, -1.0, EPS;
        RealVec<3> v(1.0, -1.0, 2.0*EPS);
        return all_close(u.head(2), v.head(2)) and
               not all_close(u.head(2), v)     and
               not all_close(u, v);
    }

    static bool _realmat_sparse() {
        RealMatN m(10, 5);
        m.setZero();
        m(2,3) = m(6,1) = m(9,4) = m(2,2) = m(3,1) = sample_normal(0, 500);
        for (Int i=0; i<m.rows(); ++i) {m(i,0) = sample_normal(100, 500);}
        for (Int j=0; j<m.cols(); ++j) {m(9,j) = sample_normal(100, 500);}
        RealMatS<Eigen::RowMajor> s; s.resize(m.rows(), m.cols());
        if (s(2,3) != 0.0) {return false;}
        try {std::cout << s(2,30) << std::endl; return false;} catch (std::out_of_range const&) {}
        for (Int i=0; i<m.rows(); ++i) {for (Int j=0; j<m.cols(); ++j) {if (m(i,j) != 0.0) {s.set_value(m(i,j), i, j);}}}
        if (not (m.transpose()*m).isApprox(s().transpose()*m)) {return false;}
        m(1,1) = sample_normal();
        s.set_value(m(1,1), 1, 1);
        if (not (m.transpose()*m).isApprox(m.transpose()*s())) {return false;}
        m(1,1) = sample_normal();
        s.initialize();
        s.set_value(m(1,1), 1, 1);
        try {s.set_value(1, 0, 1); return false;} catch (std::runtime_error const&) {}
        if (not (m.transpose()*m).isApprox(RealMatN(s().transpose()*s()))) {return false;}
        RealMatS<> r(m);
        if (not (m.transpose()*r()).isApprox(s().transpose()*m)) {return false;}
        if (r.is_initialized()) {return false;}
        r.set_block(m, 0, 0);
        if (r().nonZeros() != m.size()) {return false;}
        if (not (RealMatN(r().transpose()*r())).isApprox(s().transpose()*m)) {return false;}
        RealMatS<> x(3, 3);
        RealVec<3> u(sample_normal(0, 500), sample_normal(0, 500), sample_normal(0, 500));
        RealVec<3> v(sample_normal(0, 500), sample_normal(0, 500), sample_normal(0, 500));
        x.set_skewsym(u, 0, 0);
        if (not (x()*v).isApprox(u.cross(v))) {return false;}
        if (x().nonZeros() != 6) {return false;}
        x.set_diagonal<3>(RealVec<3>::Zero(), 0, 0).initialize();
        if (x().nonZeros() != 9) {return false;}
        return true;
    }

    /////////////////////////

    static bool _amod() {
        Real tol = 1e-12;
        Real x = sample_uniform(-1102, 192.2);
        Real a = amod(x);
        return is_close(amod(0.0), 0.0)               and
               is_close(amod(-4*PI), 0.0)             and
               is_close(amod(-14.5*PI), -PI/2.0, tol) and
               is_close(amod(32, true), 32)           and
               is_close(amod(185, true), -175)        and
               is_close(sin(a), sin(x), tol)            and
               (-PI <= a) and (a <= PI);
    }

    static bool _qexplog() {
        Real tol = 1e-6;
        Quat q = arbitrary_quat();
        Eigen::AngleAxisd aa(q);
        return is_close(q.norm(), 1.0, tol)                                            and
               all_close(qexp(qlog(q)).coeffs(), q.coeffs(), tol)                      and
               all_close(qexp(aa.angle()*aa.axis()).coeffs(), Quat(aa).coeffs(), tol)  and
               all_close(qlog(q), aa.angle()*aa.axis(), tol)                           and
               all_close(qexp(RealVec<3>::Zero()).coeffs(), Quat::Identity().coeffs()) and
               all_close(qlog(Quat::Identity()), RealVec<3>::Zero());
    }

    static bool _cross() {
        Real tol = 1e-9;
        RealVec<3> u(sample_normal(), sample_normal(), sample_normal());
        RealVec<3> v(sample_normal(), sample_normal(), sample_normal());
        return all_close(crossmat(u)*v, u.cross(v), tol) and
               all_close(crossvec(crossmat(v)), v);
    }

    static bool _euler() {
        Real tol = 1e-6;
        Quat q0(0.8006408, 0.4803845, 0.3202563, 0.1601282); Euler e0(q0);
        Quat q = arbitrary_quat(); Euler e(q, true);
        return is_close(e0.deg2rad().roll(), 1.2055949, tol) and is_close(e0.pitch(), 0.3671688, tol) and is_close(e0.yaw(), 0.6489956, tol) and
               is_close(Euler(q, false).deg2rad().deg2rad().rad2deg().amod().pitch(), Euler(e).pitch(), tol)                               and
               all_close(e.rad2deg().quat().coeffs(), q.coeffs(), tol)                                                                       and
               all_close(e.quat().coeffs(), q.coeffs(), tol);
    }

    /////////////////////////

    static bool _geoid() {
        bool success = true;
        Real tol = 1e-2; // [m]
        LLA home(sample_uniform(-PI, PI), sample_uniform(-PI, PI), sample_uniform(-100.0, 100.0));
        Geoid geoid(home);
        RealVec<3> pos(sample_uniform(-100.0, 100.0), sample_uniform(-100.0, 100.0), sample_uniform(-100.0, 100.0)); // [m, m, m]
        // Verify all inverse operations are actually inverses
        success &= pos.isApprox(geoid.ecef_from_enu(geoid.enu_from_ecef(pos)), tol);
        success &= pos.isApprox(geoid.enu_from_lla(geoid.lla_from_enu(pos)), tol);
        success &= pos.isApprox(geoid.ned_from_enu(geoid.enu_from_ned(pos)), tol);
        success &= pos.isApprox(geoid.ecef_from_ned(geoid.ned_from_ecef(pos)), tol);
        success &= pos.isApprox(geoid.ned_from_lla(geoid.lla_from_ned(pos)), tol);
        // Test against some known data pairs from http://www.oc.nps.edu/oc2902w/coord/llhxyz.htm
        Seq<LLA> test_llas;
        test_llas.push_back(LLA( DEG2RAD*31.0, DEG2RAD*-14.0,  135.0));
        test_llas.push_back(LLA(DEG2RAD*-53.0,  DEG2RAD*90.0,  -40.0));
        test_llas.push_back(LLA( DEG2RAD*89.0, DEG2RAD*132.0, 2000.0));
        Seq<RealVec<3>> test_pecefs;
        test_pecefs.push_back(RealVec<3>(5309562.0, -1323822.0,  3265963.0));
        test_pecefs.push_back(RealVec<3>(      0.0,  3846656.0, -5070512.0));
        test_pecefs.push_back(RealVec<3>( -74757.0,    83026.0,  6357777.0));
        assert(test_llas.size() == test_pecefs.size());
        for(Nat i=0; i<test_llas.size(); ++i) {
            LLA result_lla = geoid.lla_from_ecef(test_pecefs[i]);
            success &= is_close(test_llas[i].lat, result_lla.lat, 1e-4);
            success &= is_close(test_llas[i].lon, result_lla.lon, 1e-4);
            success &= is_close(test_llas[i].alt, result_lla.alt, 1.0);
            success &= test_pecefs[i].isApprox(geoid.ecef_from_lla(test_llas[i]), tol);
        }
        return success;
    }

    /////////////////////////

    static bool _uniform() {
        Real x = sample_uniform(-100.0, 100.0);
        Real y = sample_uniform(-100.0, 100.0);
        return (-100.0 <= x) and (x < 100.0) and (x != y);
    }

    static bool _normal() {
        Real m0 = sample_uniform(-10.0, 10.0); Real s0 = sample_uniform(0.0, 5.0);
        if (sample_normal(m0, s0) == sample_normal(m0, s0)) {return false;}
        if (sample_normal(m0, 0.0) != m0) {return false;}
        Int n = 10000; Real tol = 1e-1;
        Real m = 0.0; Real s = 0.0;
        for (Int i=0; i<n; ++i) {
            Real x = sample_normal(m0, s0);
            m += x; s += pow(x-m0, 2);
        }
        m /= n; s = sqrt(s/n);
        return is_close(m, m0, tol) and is_close(s, s0, tol);
    }

    static bool _bernoulli() {
        Int n = 10000; Real tol = 1e-1;
        Real p0 = sample_uniform(); Int c = 0;
        for (Int i=0; i<n; ++i) {
            c += sample_bernoulli(p0);
        }
        return is_close(Real(c)/n, p0, tol);
    }

    static bool _gaussian() {
        Int const n = 100; Real tol = 1e-6;
        RealVec<2> mean(1.0, 0.0);
        RealMat<2,2> covar;
        covar << 1.0, 2.0,
                 2.0, 4.0; // corr = 2/sqrt(1*4) = 1 => linear relation
        Gaussian<2> gaussian(mean, covar);
        RealMat<2,n> samples = gaussian.sample(n);
        for (Int i=0; i<n; ++i) {
            RealVec<2> v = samples.col(i) - mean;
            if (not is_close(v(1)/v(0), covar(0,1), tol)) {return false;}
        }
        if (all_close(samples.col(0), samples.col(1))) {return false;}
        try {Gaussian<2>(mean, covar, true);} catch (std::runtime_error const&) {return true;} // should be degenerate
        return false;
    }

    /////////////////////////

public:
    bool run() {
        bool overall_success = true;
        for (auto const& name_test : tests) {
            bool success = (*name_test.second)();
            if (not success) {std::cout << "\033[1;31mTOOLS TEST: "+name_test.first+" failed\033[0m" << std::endl;}
            overall_success &= success;
        }
        std::cout << (overall_success ? "\033[1;32m" : "\033[1;31m") << "TOOLS TEST: complete\033[0m" << std::endl;
        return overall_success;
    }
};

/////

Dict<bool (*)()> const Tests::tests({
    {"sinc", &Tests::_sinc},
    {"imod", &Tests::_imod},
    {"clip", &Tests::_clip},
    {"is_close", &Tests::_is_close},
    {"all_close", &Tests::_all_close},
    {"realmat_sparse", &Tests::_realmat_sparse},
    {"amod", &Tests::_amod},
    {"qexplog", &Tests::_qexplog},
    {"cross", &Tests::_cross},
    {"euler", &Tests::_euler},
    {"geoid", &Tests::_geoid},
    {"uniform", &Tests::_uniform},
    {"normal", &Tests::_normal},
    {"bernoulli", &Tests::_bernoulli},
    {"gaussian", &Tests::_gaussian},
});

/////

} // namespace tools
} // namespace unitair

//////////////////////////////////////////////////

/// Entry point
int main() {
    std::srand(time(nullptr));
    return !unitair::tools::Tests().run();
}

//////////////////////////////////////////////////
