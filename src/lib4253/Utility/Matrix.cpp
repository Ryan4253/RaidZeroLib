#include "main.h"

template<typename T>
Matrix<T>::Matrix(int r, int c, const T& init) {
  mat.resize(r);
  for (int i = 0; i < mat.size(); i++) {
    mat[i].resize(c, init);
  }
  row = r;
  col = c;
}

template<typename T>
Matrix<T>::Matrix(int r, int c){
  mat.resize(r);
  for (int i = 0; i < mat.size(); i++) {
    mat[i].resize(c);
  }
  row = r;
  col = c;
}

template<typename T>
Matrix<T>::Matrix(int sz, const T& init){
  Matrix(sz, sz, init);
}

template<typename T>
Matrix<T>::Matrix(int sz){
  Matrix(sz, sz);
}

template<typename T>
Matrix<T>::Matrix(const std::vector<std::vector<T> >& val){
  mat = val;
  int mxSize = 0;
  for(int i = 0; i < mat.size(); i++){
    mxSize = max(mxSize, mat[i].size());
  }

  for(int i = 0; i < mat.size(); i++){
    mat[i].resize(mxSize);
  }

  row = mat.size();
  col = mxSize;
}

template<typename T>
Matrix<T>::~Matrix(){}

template<typename T>
Matrix<T>& Matrix<T>::operator=(const Matrix<T>& rhs){
  if(&rhs == this){
    return *this;
  }

  this->mat.resize(rhs.getRow(), rhs.getCol());
  for(int i = 0; i < mat.size(); i++){
    for(int j = 0; j < mat[i].size(); i++){
      this->mat[i][j] = rhs(i, j);
    }
  }

  this->row = rhs.getRow();
  this->col = rhs.getCol();

  return *this;
}

template<typename T>
Matrix<T>& Matrix<T>::operator=(const std::vector<std::vector<T> > val){
  mat = val;
  int mxSize = 0;
  for(int i = 0; i < mat.size(); i++){
    mxSize = max(mxSize, mat[i].size());
  }

  for(int i = 0; i < mat.size(); i++){
    mat[i].resize(mxSize);
  }

  row = mat.size();
  col = mxSize;

  return *this;
}

template<typename T>
Matrix<T> Matrix<T>::operator+(const Matrix<T>& rhs){
  Matrix result(row, col, 0.0);
  for(int i = 0; i < mat.size(); i++){
    for(int j = 0; j < mat[i].size(); j++){
      result(i, j) = this->mat[i][j] + rhs(i, j);
    }
  }

  return result;
}

template<typename T>
Matrix<T>& Matrix<T>::operator+=(const Matrix<T>& rhs){
  for (int i = 0; i < rhs.getRow(); i++) {
    for (int j = 0; j < rhs.getCol(); j++) {
      this->mat[i][j] += rhs(i,j);
    }
  }

  return *this;
}

template<typename T>
Matrix<T> Matrix<T>::operator-(const Matrix<T>& rhs){
  Matrix result(row, col, 0.0);
  for(int i = 0; i < mat.size(); i++){
    for(int j = 0; j < mat[i].size(); j++){
      result(i, j) = this->mat[i][j] - rhs(i, j);
    }
  }

  return result;
}

template<typename T>
Matrix<T>& Matrix<T>::operator-=(const Matrix<T>& rhs){
  for (int i = 0; i < rhs.getRow(); i++){
    for (int j = 0; j < rhs.getCol(); j++){
      this->mat[i][j] -= rhs(i,j);
    }
  }

  return *this;
}

template<typename T>
Matrix<T> Matrix<T>::operator*(const Matrix<T>& rhs){
  Matrix result(rhs.getRow(), rhs.getCol(), 0.0);

  for (int i = 0; i < rhs.getRow(); i++){
    for (int j = 0; j < rhs.getCol(); j++){
      for (int k = 0; k < rhs.getRow(); k++){
        result(i,j) += this->mat[i][k] * rhs(k,j);
      }
    }
  }

  return result;
}

template<typename T>
Matrix<T>& Matrix<T>::operator*=(const Matrix<T>& rhs){
  Matrix result = (*this) * rhs;
  *this = result;
  return *this;
}

template<typename T>
Matrix<T> Matrix<T>::transpose(){
  Matrix result(rows, cols, 0.0);

  for(int i = 0; i < row; i++){
    for(int j = 0; j < col; j++){
      result(i, j) = this->mat[j][i];
    }
  }

  return result;
}

template<typename T>
Matrix<T>& Matrix<T>::resize(int r, int c){
  mat.resize(r);
  for(int i = 0; i < mat.size(); i++){
    mat[i].resize(c);
  }

  return *this;
}

template<typename T>
Matrix<T> Matrix<T>::operator+(const T& rhs){
  Matrix result(row, col, 0.0);

  for (int i = 0; i < row; i++) {
    for (int j = 0; j < col; j++) {
      result(i,j) = this->mat[i][j] + rhs;
    }
  }

  return result;
}

template<typename T>
Matrix<T> Matrix<T>::operator-(const T& rhs){
  Matrix result(row, col, 0.0);

  for (int i = 0; i < row; i++) {
    for (int j = 0; j < col; j++) {
      result(i,j) = this->mat[i][j] - rhs;
    }
  }

  return result;
}

template<typename T>
Matrix<T> Matrix<T>::operator*(const T& rhs){
  Matrix result(row, col, 0.0);

  for (int i = 0; i < row; i++) {
    for (int j = 0; j < col; j++) {
      result(i,j) = this->mat[i][j] * rhs;
    }
  }

  return result;
}

template<typename T>
Matrix<T> Matrix<T>::operator/(const T& rhs){
  Matrix result(row, col, 0.0);

  for (int i = 0; i < row; i++) {
    for (int j = 0; j < col; j++) {
      result(i,j) = this->mat[i][j] / rhs;
    }
  }

  return result;
}

template<typename T>
Matrix<T> Matrix<T>::operator*(const std::vector<T>& rhs){
  std::vector<T> result(rhs.size(), 0.0);

  for (int i = 0; i < row; i++) {
    for (int j = 0; j < col; j++) {
      result[i] = this->mat[i][j] * rhs[j];
    }
  }

  return result;
}

template<typename T>
std::vector<T> Matrix<T>::getDiag(){
  std::vector<T> result(min(row, col), 0.0);

  for (int i = 0; i < min(row, col); i++) {
    result[i] = this->mat[i][i];
  }

  return result;
}

template<typename T>
T& Matrix<T>::operator()(int r, int c){
  return this->mat[r][c];
}

template<typename T>
const T& Matrix<T>::operator()(int r, int c) const{
  return this->mat[r][c];
}

template<typename T>
int Matrix<T>::getRow() const{
  return this->row;
}

int Matrix<T>::getCol() const{
  return this->col;
}
