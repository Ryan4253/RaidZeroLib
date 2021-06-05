#pragma once
#include<vector>

namespace lib4253{
template<typename T> class Matrix{
    private:
    std::vector<std::vector<T> > mat;
    int row, col;

    public:
    //  constructors
    Matrix(const std::vector<std::vector<T> >& val);
    Matrix(int sz);
    Matrix(int sz, const T& init);
    Matrix(int r, int c);
    Matrix(int r, int c, const T& init);
    virtual ~Matrix();
    
    //  Matrix Operations
    Matrix<T>& operator=(const Matrix<T>& rhs);
    Matrix<T>& operator=(std::vector<std::vector<T> > rhs);
    Matrix<T> operator+(const Matrix<T>& rhs);
    Matrix<T>& operator+=(const Matrix<T>& rhs);
    Matrix<T> operator-(const Matrix<T>& rhs);
    Matrix<T>& operator-=(const Matrix<T>& rhs);
    Matrix<T> operator*(const Matrix<T>& rhs);
    Matrix<T>& operator*=(const Matrix<T>& rhs);
    Matrix<T> transpose();
    Matrix<T>& resize(int r, int c);
    
    //  Scalar Operations
    Matrix<T> operator+(const T& rhs);
    Matrix<T> operator-(const T& rhs);
    Matrix<T> operator*(const T& rhs);
    Matrix<T> operator/(const T& rhs);
    
    //  Vector Operations
    Matrix<T> operator*(const std::vector<T>& rhs);
    std::vector<T> getDiag();
    
    //  Getter
    T& operator()(int r, int c);
    const T& operator()(int r, int c) const;
    int getRow() const;
    int getCol() const;
};
}