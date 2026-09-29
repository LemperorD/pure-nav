#include <memory>

namespace pure::common {

class ShMWriter
{
public:

};

class ShMReader
{
public:
	ShMReader(std::shared_ptr<void> shm_ptr) : shm_ptr_(std::move(shm_ptr)) {}

private:
	std::shared_ptr<void> shm_ptr_;
};

} // namespace pure::common