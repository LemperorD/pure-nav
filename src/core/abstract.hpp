#ifndef ABSTRACT_HPP
#define ABSTRACT_HPP

#include <memory>

class IProduct {
public:
    virtual ~IProduct() = default;
    virtual void init() = 0;
    virtual double getFreq() = 0;
private:
    double freq_;
};

class IFactory {
public:
    virtual ~IFactory() = default;
    virtual std::unique_ptr<IProduct> createProduct() = 0;
};

#endif // ABSTRACT_HPP