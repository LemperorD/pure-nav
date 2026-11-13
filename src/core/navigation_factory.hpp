#ifndef NAVIGATION_FACTORY_HPP
#define NAVIGATION_FACTORY_HPP

#include <cstdint>
#include <thread>

#include "abstract.hpp"
#include "nav_ds.hpp"

class INavigationProduct : public IProduct
{
    // Additional methods specific to navigation products can be added here
};

class INavigationFactory : public IFactory
{
public:
    std::unique_ptr<IProduct> createProduct() override{
        return std::unique_ptr<INavigationProduct>();
    };
};

class ISlam : public INavigationProduct
{
public:
    enum class SlamMode : std::uint8_t { MAP = 0, NAV = 1 };
    SlamMode getMode() const { return mode_; }
    virtual void localize() = 0;
    virtual void map() = 0;
private:
    SlamMode mode_;
};

class IPlan : public INavigationProduct
{
public:
    virtual void global_plan() = 0;
    virtual void local_plan() = 0;
private:
    Trajectory global_plan_;
    Trajectory local_plan_
};

class IControl : public INavigationProduct
{
public:
    enum class CtrlAlg : std::uint8_t { PID = 0, MPC = 1 };
    CtrlAlg getCtrlAlg() const { return ctrl_alg_; }

    virtual void control() = 0;
private:
    CtrlAlg ctrl_alg_;
};

#endif // NAVIGATION_FACTORY_HPP