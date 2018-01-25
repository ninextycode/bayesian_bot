package bayes.additional;

import java.lang.System;
import java.util.Arrays;
import java.lang.ArrayIndexOutOfBoundsException;
import java.lang.IllegalArgumentException;

import java.lang.Exception;
import java.lang.StringBuffer;
import java.lang.Math;

public class Matrix {
	public static void main(String[] args) {
		test();
	}

	public static void test() {
		Matrix result;

		Matrix a = new Matrix(2, 3, new double[]
				{
						4.5, 6.5, 3.5,
						7.5, 5,  8.5
				});

		Matrix b = new Matrix( 3, 4, new double[] {
		 1, 5, -1, 2,
		 6, -2,  3,  7,
		 -3,  4,8,-4
		});

		Matrix r = new Matrix(2, 4, new double[] {
				33, 23.5, 43, 40.5,
				12, 61.5, 75.5, 16
				});
		result = a.multiply(b);

		System.out.println("Matrix multiplication");
		System.out.println("Expected: " + r.toStringFull());
		System.out.println("Actual: " + result.toStringFull());
		System.out.println("Test passed:                " +  result.equals(r)  + "\n");

		Matrix r3 = new Matrix(2, 4, new double[] {
				99,  70.5, 129.0, 121.5,
				36, 184.5, 226.5,  48.0
		});
		result = r.scale(3);
		System.out.println("Scale by constant");
		System.out.println("Expected: " + r3.toStringFull());
		System.out.println("Actual: " + result.toStringFull());
		System.out.println("Test passed:                " +  result.equals(r3)  + "\n");

		Matrix rt = new Matrix(4, 2, new double[] {
				33, 12,
				23.5, 61.5,
				43, 75.5,
				40.5, 16 });
		result = r.T();
		System.out.println("Transpose");
		System.out.println("Expected: " + rt.toStringFull());
		System.out.println("Actual: " + result.toStringFull());
		System.out.println("Test passed:                " +  r.T().equals(rt) + "\n");

		Matrix three = new Matrix(2, 4, 3);
		result = r.scale(three);
		System.out.println("Scale by matrix, default value matrix");
		System.out.println("Expected: " + r3.toStringFull());
		System.out.println("Actual: " + result.toStringFull());
		System.out.println("Test passed:                " +  result.equals(r3)  + "\n");

		System.out.println("Sequential add(and subtract) matrix, scale by constant");
		result = r.add(r).add(r).add(r).add(r.scale(-1));
		System.out.println("Expected: " + r3.toStringFull());
		System.out.println("Actual: " + result.toStringFull());
		System.out.println("Test passed:                " +  result.equals(r3)  + "\n");

		Matrix r05 = new Matrix(2, 4, new double[] {
				33.5, 24, 43.5, 41,
				12.5, 62, 76, 16.5
		});

		result = r.add(0.5);
		System.out.println("Add constant");
		System.out.println("Expected: " + r05.toStringFull());
		System.out.println("Actual: " + result.toStringFull());
		System.out.println("Test passed:                " +  result.equals(r05)  + "\n");

		Matrix x = new Matrix(5, 1, new double[] { -2, -1, 0, 1, 2 });
		result = x.map(new Mapper() {
			public double map(double x) {
				return Math.pow(2, x);
			}
		});

		Matrix expected = new Matrix(5, 1, new double[] { 1/4f, 1/2f, 1, 2, 4 });
		System.out.println("Apply function");
		System.out.println("Expected: " + expected.toStringFull());
		System.out.println("Actual: " + result.toStringFull());
		System.out.println("Test passed:                " +  result.equals(expected)  + "\n");

		Matrix m = new Matrix(2, 2, new double[] { 1/4f, -1/2f, -2, 4 });
		double prod = m.reduce(new Reducer() {
			public final double getIdentity() { return 1; };
			public double reduce(double a, double b) {
				return a*b;
			}
		});

		double sum = m.reduce(new Reducer() {
			public final double getIdentity() { return 0; };
			public double reduce(double a, double b) {
				return a+b;
			}
		});

		double expSum = 1.75;
		double expProd = 1;

		System.out.println("Reduce");
		System.out.println("Expected sum: " + expSum);
		System.out.println("Actual sum: " + sum);
		System.out.println("Expected product: " + expProd);
		System.out.println("Actual product: " + prod);
		System.out.println("Test passed:                " +  ((expSum == sum) && (expProd == prod)) + "\n");
	}

	public static boolean doRowsMatch(Matrix a, int r) {
		return (a.rows() == r);
	}

	public static boolean doColsMatch(Matrix a, int c) {
		return (a.cols() == c);
	}

	public static boolean doRowsMatch(Matrix a, Matrix b) {
		return doRowsMatch(a, b.rows());
	}

	public static boolean doColsMatch(Matrix a, Matrix b) {
		return doColsMatch(a, b.cols());
	}

	public static boolean doMultiplyDimMatch(Matrix a, Matrix b, Matrix out) {
		return a.cols() == b.rows() && a.rows() == out.rows() && out.cols() == b.cols();
	}

	private static boolean do1dLength(int length, int rows, int cols) {
		return ((rows * cols) == length);
	}

	public static boolean doDimMatch(Matrix a, Matrix b) {
		return doRowsMatch(a, b) && doColsMatch(a, b);
	}

	private double[] data;
	private boolean by_columns = false;

	private int _rows;

	public int rows() {
		return _rows;
	}

	private int _cols;

	public int cols() {
		return _cols;
	}


	public Matrix(int rows, int cols) {
		this(rows, cols, 0d);
	}

	public Matrix(int rows, int cols, double defaultValue) {
		this.data = new double[rows * cols];
		Arrays.fill(data, defaultValue);
		_rows = rows;
		_cols = cols;
	}

	public Matrix(int rows, int cols, double[] data) {
		if (!do1dLength(data.length, rows, cols)) {
			throw new IllegalArgumentException("1d length,  " + data.length + " != rows * cols, " + rows * cols);
		}
		_rows = rows;
		_cols = cols;
		this.data = new double[rows * cols];
		System.arraycopy(data, 0, this.data, 0, data.length);
	}

	public Matrix(int rows, int cols, double[] data, boolean by_coluns) {
		this(rows, cols, data);
		this.by_columns = by_coluns;
	}

	public Matrix multiply(Matrix m) {
		int newRows = this.rows();
		int newCols = m.cols();
		Matrix newMatrix = new Matrix(newRows, newCols, 0);
		return Matrix.multiply(this, m, newMatrix);
	}



	public Matrix multiply(Matrix m, Matrix out) {
		return Matrix.multiply(this, m, out);
	}

	public static Matrix multiply(Matrix m1, Matrix m2, Matrix out) {
		if (!doMultiplyDimMatch(m1, m2, out)) {
			throw new IllegalArgumentException("Dimentions dont match for multiplication,  " +
												m1 + " " + m2 + " , output: "  + out);
		}

		int newRows = m1.rows();
		int newCols = m2.cols();

		for (int r = 0; r < newRows; r++) {
			for (int c = 0; c < newCols; c++) {
				out.set(r, c, valueAfterMultiply(m1, m2, r, c));
			}
		}

		return out;
	}

	private static double valueAfterMultiply(Matrix m1, Matrix m2, int r, int c) {
		int innerDim = m1.cols();
		double t = 0;
		for (int k = 0; k < innerDim; k++) {
			t += (m1.get(r, k) * m2.get(k, c));
		}
		return t;
	}


	public Matrix scale(Matrix m) {
		Matrix newMatrix = this.copy();
		return newMatrix.inplaceScale(m);
	}

	public Matrix scale(double a) {
		Matrix newMatrix = this.copy();
		return newMatrix.inplaceScale(a);
	}

	public Matrix inplaceScale(Matrix m) {
		if (!Matrix.doDimMatch(m, this)) {
			throw new IllegalArgumentException("Dimentions dont match, " + this + ", " + m);
		}

		for (int c = 0; c < cols(); c++) {
			for (int r = 0; r < rows(); r++) {
				this.set(r, c, this.get(r, c) * m.get(r, c));
			}
		}
		return this;
	}

	public Matrix inplaceScale(double a) {
		for (int c = 0; c < cols(); c++) {
			for (int r = 0; r < rows(); r++) {
				this.set(r, c, this.get(r, c) * a);
			}
		}
		return this;
	}

	public Matrix add(Matrix m) {
		Matrix newMatrix = this.copy();
		return newMatrix.inplaceAdd(m);
	}

	public Matrix add(double a) {
		Matrix newMatrix = this.copy();
		return newMatrix.inplaceAdd(a);
	}

	public Matrix inplaceAdd(Matrix m) {
		if (!Matrix.doDimMatch(m, this)) {
			throw new IllegalArgumentException("Dimentions dont match, " + this + ", " + m);
		}
		for (int c = 0; c < cols(); c++) {
			for (int r = 0; r < rows(); r++) {
				this.set(r, c, this.get(r, c) + m.get(r, c));
			}
		}
		return this;
	}

	public Matrix inplaceAdd(double a) {
		for (int c = 0; c < cols(); c++) {
			for (int r = 0; r < rows(); r++) {
				this.set(r, c, this.get(r, c) + a);
			}
		}
		return this;
	}

	public double reduce(Reducer reducer) {
		double answer = reducer.getIdentity();

		for(int r = 0; r < rows(); r++) {
			for(int c = 0; c < cols(); c++) {
				answer = reducer.reduce(answer, get(r, c));
			}
		}
		return answer;
	}

	public Matrix map(Mapper mapper) {
		Matrix newMatrix = this.copy();
		return newMatrix.inplaceMap(mapper);
	}

	public Matrix inplaceMap(Mapper mapper) {
		for (int c = 0; c < cols(); c++) {
			for (int r = 0; r < rows(); r++) {
				set(r, c, mapper.map(get(r, c)));
			}
		}
		return this;
	}

	public Matrix T() {
		Matrix newMatrix = this.copy();
		return newMatrix.inplaceT();
	}

	public Matrix inplaceT() {
		by_columns = !by_columns;
		int t = _rows;
		_rows = _cols;
		_cols = t;

		return this;
	}

	public Matrix copy() {
		return new Matrix(rows(), cols(), data, by_columns);
	}

	private int get1dIndex(int r, int c) {
		return by_columns? rows() * c + r: cols() * r + c;
	}

	public double get(int r, int c) {
		if(!(0 <= r && r < rows() && 0 <= c && c < cols())) {
			throw new ArrayIndexOutOfBoundsException("trying to get (" + r + ", " + c + ") from " + this);
		}
		int ind = get1dIndex(r, c);
		return data[ind];
	}

	public double get(int r, int c, double def) {
		if(0 <= r && r < rows() && 0 <= c && c < cols()) {
			return get(r, c);
		} else {
			return def;
		}

	}

	public boolean inMatrix(int r, int c) {
		return (0 <= r && r < rows() && 0 <= c && c < cols());
	}

	public double set(int r, int c, double val) {
		if(!(0 <= r && r < rows() && 0 <= c && c < cols())) {
			throw new ArrayIndexOutOfBoundsException("trying to set (" + r + ", " + c + "), " + this);
		}
		int ind = get1dIndex(r, c);
		data[ind] = val;
		return val;
	}

	@Override
	public String toString() {
		return "Matrix (" + rows() + "," + cols() + ")";
	}

	@Override
	public boolean equals(Object o) {
		if (o instanceof Matrix) {
			Matrix m = (Matrix) (o);
			if (!Matrix.doDimMatch(this, m)) {
				return false;
			}
			for (int r = 0; r < rows(); r++) {
				for (int c = 0; c < cols(); c++) {
					if (get(r, c) != m.get(r, c)) {
						return false;
					}
				}
			}
			return true;
		}
		return false;
	}

	public String toStringFull() {
		String s = toString() + "\n";
		StringBuffer sb = new StringBuffer();
		sb.append(s);

		for (int r = 0; r < rows(); r++) {
			StringBuffer sbRow = new StringBuffer();
			sbRow.append(get(r, 0));
			for (int c = 1; c < cols(); c++) {
				sbRow.append("\t" + get(r, c));
			}
			sbRow.append("\n");
			sb.append(sbRow.toString());
		}
		return sb.toString();
	}
}
