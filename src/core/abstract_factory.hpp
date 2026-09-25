#include <memory>

class INavigationFactory {
public:
    virtual ~INavigationFactory() = default;
    virtual std::unique_ptr<INavigation> createNavigation() = 0;
};