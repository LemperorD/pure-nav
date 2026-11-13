#ifndef NAVIGATION_FACTORY_HPP
#define NAVIGATION_FACTORY_HPP

#include "abstract.hpp"

class INavigationProduct : public IProduct {
public:
    virtual void navigate() = 0;
};

class INavigationFactory : public IFactory {
public:
    std::unique_ptr<IProduct> createProduct() override{
        return std::unique_ptr<INavigationProduct>(new INavigationProduct());
    };
};

class ILocalize : public INavigationProduct {
public:
    virtual void localize() = 0;
};

class IPlan : public INavigationProduct {
public:
    virtual void plan() = 0;
};

class IControl : public INavigationProduct {
public:
    virtual void control() = 0;
};



#endif // NAVIGATION_FACTORY_HPP