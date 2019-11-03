#include "TestSE3.h"

namespace controller {
	using namespace Jama;

	void TestSE3::main(std::string args[]) throw(IOException) {
		/*SE3 c = new SE3(null);
		
		Matrix M0 = new Matrix(new double[][]{{1, 2, 3}, {4, 5, 6}});
		System.out.println("rows: " + M0.getRowDimension() + "\n");
		System.out.println("cols: " + M0.getColumnDimension() + "\n");
		System.out.println("M0 = \n" + toString(M0));
		
		Matrix M1 = new Matrix(new double[]{1, 2, 3}, 1);
		Matrix M2 = new Matrix(new double[]{4, 5, 6}, 1);
		System.out.println("M1 = \n" + toString(M1));
		System.out.println("M2 = \n" + toString(M2));
		System.out.println("M1 x M2 = \n" + toString(c.crossProduct(M1, M2)));
		
		System.out.println("M1^ = \n" + toString(c.hatMap(M1)));
		System.out.println("M1^ = \n" + toString(c.veeMap(c.hatMap(M1))));
		
		System.out.println("R = \n" + toString(c.getRotationMatrix(0, 0, Math.PI/2)));*/

		/*File file = new File("C:\\Users\\Andriy\\Desktop\\rules.txt");
		file.createNewFile();
		FileWriter fw = new FileWriter(file.getAbsoluteFile());
		BufferedWriter bw = new BufferedWriter(fw);
		
		ArrayList<String> proportionalError = new ArrayList<String>();
		proportionalError.add("bigNegativeProportionalError");
		proportionalError.add("smallNegativeProportionalError");
		proportionalError.add("noProportionalError");
		proportionalError.add("smallPositiveProportionalError");
		proportionalError.add("bigPositiveProportionalError");
		
		ArrayList<String> integralError = new ArrayList<String>();
		integralError.add("bigNegativeIntegralError");
		integralError.add("smallNegativeIntegralError");
		integralError.add("noIntegralError");
		integralError.add("smallPositiveIntegralError");
		integralError.add("bigPositiveIntegralError");
		
		ArrayList<String> derivativeError = new ArrayList<String>();
		derivativeError.add("bigNegativeDerivativeError");
		derivativeError.add("smallNegativeDerivativeError");
		derivativeError.add("noDerivativeError");
		derivativeError.add("smallPositiveDerivativeError");
		derivativeError.add("bigPositiveDerivativeError");
		
		ArrayList<String> desiredVelocity = new ArrayList<String>();
		desiredVelocity.add("veryHighNegativeDesiredVelocity");
		desiredVelocity.add("highNegativeDesiredVelocity");
		desiredVelocity.add("mediumNegativeDesiredVelocity");
		desiredVelocity.add("lowNegativeDesiredVelocity");
		desiredVelocity.add("noDesiredVelocity");
		desiredVelocity.add("lowPositiveDesiredVelocity");
		desiredVelocity.add("mediumPositiveDesiredVelocity");
		desiredVelocity.add("highPositiveDesiredVelocity");
		desiredVelocity.add("veryHighPositiveDesiredVelocity");
		
		for(int indexIntegralError = 0; indexIntegralError < integralError.size(); ++indexIntegralError)
			for(int indexDerivativeError = 0; indexDerivativeError < derivativeError.size(); ++indexDerivativeError)
				for(int indexProportionalError = 0; indexProportionalError < proportionalError.size(); ++indexProportionalError)
					bw.write("rulebase.addRule(new T1_Rule(new T1_Antecedent[]{" + integralError.get(indexIntegralError) + "," + derivativeError.get(indexDerivativeError) + "," + proportionalError.get(indexProportionalError) + "}, ));\n");
		
		bw.close();*/

	}

	std::string TestSE3::toString(Matrix* M) {
		std::string s = "[";

		for(int i = 0; i < M->getRowDimension(); ++i) {
			for(int j = 0; j < M->getColumnDimension(); ++j) {
				s += M->get(i, j);
				if(j + 1 < M->getColumnDimension())
					s += std::string(", ");
			}
			if(i + 1 < M->getRowDimension())
				s += std::string(";\n");
			else
				s += std::string("]");
		}

		return s;
	}
}
