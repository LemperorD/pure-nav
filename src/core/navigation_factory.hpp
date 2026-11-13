#ifndef NAVIGATION_FACTORY_HPP
#define NAVIGATION_FACTORY_HPP

#include "abstract.hpp"


class ILocalize : public IProduct {
public:
    virtual void localize() = 0;
};

class IPlan : public IProduct {
public:
    virtual void plan() = 0;
};

class IControl : public IProduct {
public:
    virtual void control() = 0;
};

class INavigationFactory : public IFactory {
public:
    virtual std::unique_ptr<IProduct> createNavigationProduct() = 0;
};

#endif // NAVIGATION_FACTORY_HPP