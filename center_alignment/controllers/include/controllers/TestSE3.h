#pragma once

#include <string>

namespace controller {


	using namespace Jama;

	class TestSE3 {
		static void main(std::string args[]) throw(IOException);

	public:
		static std::string toString(Matrix* M);
	};
}
