#pragma once
#pragma once
#pragma once
#include<iostream>
#include<fstream>
#include<string>
#include<vector>
#include <iomanip>
#include <search.h>
#include <algorithm>
#include<cmath>
#include<sstream>
#include <limits>
#include <stdexcept>  // 异常处理
#include <initializer_list>  // 初始化列表
using namespace std;

class Matrix {//缩略该class可以看到矩阵用法说明
private:
    size_t rows_, cols_;
    vector<double> data_;  // 一维数组存储，提高缓存命中率

public:
    // 构造函数（指定维度，可选初始化列表）
    Matrix(size_t rows = 0, size_t cols = 0,
        initializer_list<initializer_list<double>> init = {})
        : rows_(rows), cols_(cols), data_(rows* cols, 0.0) {

        // 如果有初始化列表，则使用它来初始化
        if (init.size() > 0) {
            if (init.size() != rows_ || init.begin()->size() != cols_) {
                throw invalid_argument("初始化列表维度与指定维度不匹配！");
            }

            size_t index = 0;
            for (const auto& row : init) {
                if (row.size() != cols_) {
                    throw invalid_argument("所有行的列数必须相同！");
                }
                for (double val : row) {
                    data_[index++] = val;
                }
            }
        }
    }

    // 保留原来的初始化列表构造函数
    Matrix(initializer_list<initializer_list<double>> init) {
        rows_ = init.size();
        cols_ = (rows_ == 0) ? 0 : init.begin()->size();
        data_.resize(rows_ * cols_);

        size_t index = 0;
        for (const auto& row : init) {
            if (row.size() != cols_) {
                throw invalid_argument("所有行的列数必须相同！");
            }
            for (double val : row) {
                data_[index++] = val;
            }
        }
    }
    // 获取行数
    size_t rows() const { return rows_; }

    // 获取列数
    size_t cols() const { return cols_; }

    // 访问元素（可修改）
    double& operator()(size_t row, size_t col) {
        if (row >= rows_ || col >= cols_) {
            throw out_of_range("矩阵索引越界！");
        }
        return data_[row * cols_ + col];
    }

    // 访问元素（只读）
    double operator()(size_t row, size_t col) const {
        if (row >= rows_ || col >= cols_) {
            throw out_of_range("矩阵索引越界！");
        }
        return data_[row * cols_ + col];
    }
    // 生成单位矩阵
    Matrix identity(size_t n) {
        Matrix result(n, n);
        for (size_t i = 0; i < n; ++i) {
            result(i, i) = 1.0;
        }
        return result;
    }
    // 取矩阵的某一行
    Matrix row(size_t row_index) const {
        if (row_index >= rows_) {
            throw out_of_range("行索引越界！");
        }

        Matrix row_vector(1, cols_);  // 创建 1×cols_ 的行向量
        for (size_t j = 0; j < cols_; ++j) {
            row_vector(0, j) = (*this)(row_index, j);  // 复制数据
        }
        return row_vector;
    }
    // 取矩阵的某一列
    Matrix col(size_t col_index) const {
        if (col_index >= cols_) {
            throw out_of_range("列索引越界！");
        }

        Matrix col_vector(rows_, 1);  // 创建 rows_×1 的列向量
        for (size_t i = 0; i < rows_; ++i) {
            col_vector(i, 0) = (*this)(i, col_index);  // 复制数据
        }
        return col_vector;
    }
    // 矩阵加法
    Matrix operator+(const Matrix& other) const {
        if (rows_ != other.rows_ || cols_ != other.cols_) {
            throw invalid_argument("矩阵维度必须相同才能相加！");
        }

        Matrix result(rows_, cols_);
        for (size_t i = 0; i < rows_; ++i) {
            for (size_t j = 0; j < cols_; ++j) {
                result(i, j) = (*this)(i, j) + other(i, j);
            }
        }
        return result;
    }
    // 矩阵减法
    Matrix operator-(const Matrix& other) const {
        if (rows_ != other.rows_ || cols_ != other.cols_) {
            throw invalid_argument("矩阵维度必须相同才能相减！");
        }

        Matrix result(rows_, cols_);
        for (size_t i = 0; i < rows_; ++i) {
            for (size_t j = 0; j < cols_; ++j) {
                result(i, j) = (*this)(i, j) - other(i, j);
            }
        }
        return result;
    }
    // 矩阵乘法（类似Eigen的 operator*）
    Matrix operator*(const Matrix& other) const {
        if (cols_ != other.rows_) {
            throw invalid_argument("矩阵维度不匹配，无法相乘！");
        }

        Matrix result(rows_, other.cols_);
        for (size_t i = 0; i < rows_; i++) {
            for (size_t j = 0; j < other.cols_; j++) {
                double sum = 0;
                for (size_t k = 0; k < cols_; k++) {
                    sum += (*this)(i, k) * other(k, j);
                }
                result(i, j) = sum;
            }
        }
        return result;
    }
    // 矩阵转置
    Matrix transpose() const {
        Matrix result(cols_, rows_);  // 新矩阵行列互换
        for (size_t i = 0; i < rows_; ++i) {
            for (size_t j = 0; j < cols_; ++j) {
                result(j, i) = (*this)(i, j);  // 交换行列索引
            }
        }
        return result;
    }
    // 矩阵求逆（仅支持方阵）
    Matrix inverse() const {
        if (rows_ != cols_) {
            throw invalid_argument("矩阵必须是方阵才能求逆！");
        }

        size_t n = rows_;
        Matrix augmented(n, 2 * n);  // 创建增广矩阵 [A | I]

        // 初始化增广矩阵
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < n; ++j) {
                augmented(i, j) = (*this)(i, j);  // 左半部分为原矩阵
            }
            augmented(i, i + n) = 1.0;  // 右半部分为单位矩阵
        }

        // 高斯-约旦消元法
        for (size_t i = 0; i < n; ++i) {
            // 寻找主元
            size_t pivot = i;
            for (size_t j = i + 1; j < n; ++j) {
                if (fabs(augmented(j, i)) > fabs(augmented(pivot, i))) {
                    pivot = j;
                }
            }

            // 如果主元为0，矩阵不可逆
            if (fabs(augmented(pivot, i)) < 1e-10) {
                throw runtime_error("矩阵不可逆！");
            }

            // 交换行
            if (pivot != i) {
                for (size_t j = 0; j < 2 * n; ++j) {
                    swap(augmented(i, j), augmented(pivot, j));
                }
            }

            // 归一化主元行
            double divisor = augmented(i, i);
            for (size_t j = 0; j < 2 * n; ++j) {
                augmented(i, j) /= divisor;
            }

            // 消元其他行
            for (size_t k = 0; k < n; ++k) {
                if (k != i) {
                    double factor = augmented(k, i);
                    for (size_t j = 0; j < 2 * n; ++j) {
                        augmented(k, j) -= factor * augmented(i, j);
                    }
                }
            }
        }

        // 提取逆矩阵（增广矩阵的右半部分）
        Matrix result(n, n);
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < n; ++j) {
                result(i, j) = augmented(i, j + n);
            }
        }

        return result;
    }

    // 从二维数组创建矩阵
    template<size_t Rows, size_t Cols>
    static Matrix fromArray(const double(&arr)[Rows][Cols]) {
        Matrix result(Rows, Cols);
        for (size_t i = 0; i < Rows; ++i) {
            for (size_t j = 0; j < Cols; ++j) {
                result(i, j) = arr(i, 0)[j];
            }
        }
        return result;
    }

    // 从一维数组创建列向量
    template<size_t Size>
    static Matrix fromVector(const double(&arr)[Size]) {
        Matrix result(Size, 1);  // 创建 Size×1 的列向量
        for (size_t i = 0; i < Size; ++i) {
            result(i, 0) = arr(i, 0);
        }
        return result;
    }

    // 新增功能：删除全为0的行，返回新矩阵
    Matrix removeZeroRows() const {
        // 空矩阵直接返回
        if (rows_ == 0 || cols_ == 0) {
            return Matrix();
        }

        // 计算非零行的数量
        size_t nonZeroRows = 0;
        for (size_t i = 0; i < rows_; i++) {
            if (!isRowZero(i)) {
                nonZeroRows++;
            }
        }

        // 如果所有行都是0，返回空矩阵
        if (nonZeroRows == 0) {
            return Matrix();
        }

        // 创建新矩阵存储非零行
        Matrix result(nonZeroRows, cols_);
        size_t newRow = 0;

        for (size_t i = 0; i < rows_; i++) {
            if (!isRowZero(i)) {
                // 复制非零行到新矩阵
                for (size_t j = 0; j < cols_; j++) {
                    result(newRow, j) = (*this)(i, j);
                }
                newRow++;
            }
        }

        return result;
    }
    // 新增功能：删除全为0的列，返回新矩阵
    Matrix removeZeroCols() const {
        // 空矩阵直接返回
        if (rows_ == 0 || cols_ == 0) {
            return Matrix();
        }

        // 计算非零列的数量
        size_t nonZeroCols = 0;
        for (size_t j = 0; j < cols_; j++) {
            if (!isColumnZero(j)) {
                nonZeroCols++;
            }
        }

        // 如果所有列都是0，返回空矩阵
        if (nonZeroCols == 0) {
            return Matrix();
        }

        // 创建新矩阵存储非零列
        Matrix result(rows_, nonZeroCols);
        size_t newCol = 0;

        for (size_t j = 0; j < cols_; j++) {
            if (!isColumnZero(j)) {
                // 复制非零列到新矩阵
                for (size_t i = 0; i < rows_; i++) {
                    result(i, newCol) = (*this)(i, j);
                }
                newCol++;
            }
        }

        return result;
    }
    Matrix removeZeroRowsAndCols() const {
        // 先去除全零行
        Matrix matWithoutZeroRows = removeZeroRows();
        // 再在去除全零行的结果上去除全零列
        return matWithoutZeroRows.removeZeroCols();
    }

private:
    // 辅助函数：检查指定行是否全为0
    bool isRowZero(size_t rowIndex) const {
        if (rowIndex >= rows_) {
            throw out_of_range("行索引越界！");
        }

        // 定义一个小的epsilon值用于浮点数比较
        const double epsilon = 1e-10;

        for (size_t j = 0; j < cols_; j++) {
            if (fabs((*this)(rowIndex, j)) > epsilon) {
                return false;  // 发现非零元素
            }
        }

        return true;  // 所有元素都是0
    }
    // 辅助函数：检查指定列是否全为0
    bool isColumnZero(size_t colIndex) const {
        if (colIndex >= cols_) {
            throw out_of_range("列索引越界！");
        }

        // 定义一个小的epsilon值用于浮点数比较
        const double epsilon = 1e-10;

        for (size_t i = 0; i < rows_; i++) {
            if (fabs((*this)(i, colIndex)) > epsilon) {
                return false;  // 发现非零元素
            }
        }

        return true;  // 所有元素都是0
    }
public:
    // 行视图类（仅支持初始化列表赋值）
    class RowView {
    private:
        Matrix& matrix_;      // 引用原始矩阵
        size_t row_index_;    // 目标行索引

    public:
        RowView(Matrix& matrix, size_t row_index)
            : matrix_(matrix), row_index_(row_index) {}

        // 用初始化列表修改行数据
        void operator=(initializer_list<double> list) {
            if (list.size() != matrix_.cols_) {
                throw invalid_argument("初始化列表长度必须等于矩阵列数");
            }
            size_t col = 0;
            for (double val : list) {
                matrix_(row_index_, col++) = val;
            }
        }
    };

    // 获取可修改的行视图
    RowView row(size_t row_index) {
        if (row_index >= rows_) {
            throw out_of_range("行索引越界！");
        }
        return RowView(*this, row_index);
    }
    // 判断矩阵是否可以求逆
    bool isInvertible() const {
        // 1. 检查是否为方阵
        if (rows_ != cols_) {
            return false;
        }

        // 2. 检查矩阵是否为零矩阵（零矩阵不可逆）
        if (rows_ == 0) {
            return false;
        }

        // 3. 特殊情况：1x1矩阵，判断元素是否为0
        if (rows_ == 1) {
            return fabs(data_[0]) > 1e-10;
        }

        // 4. 通用情况：使用高斯消元法检查秩是否等于阶数
        Matrix temp = *this;  // 复制原矩阵，避免修改原始数据
        size_t rank = 0;

        for (size_t i = 0; i < rows_; ++i) {
            // 寻找主元（当前列中第一个非零元素）
            size_t pivot = i;
            while (pivot < rows_ && fabs(temp(pivot, i)) < 1e-10) {
                pivot++;
            }

            // 如果当前列全为0，跳过（秩不增加）
            if (pivot == rows_) {
                continue;
            }

            // 交换行
            if (pivot != i) {
                for (size_t j = 0; j < cols_; ++j) {
                    swap(temp(i, j), temp(pivot, j));
                }
            }

            // 消元其他行
            for (size_t k = 0; k < rows_; ++k) {
                if (k != i && fabs(temp(k, i)) > 1e-10) {
                    double factor = temp(k, i) / temp(i, i);
                    for (size_t j = i; j < cols_; ++j) {
                        temp(k, j) -= factor * temp(i, j);
                    }
                }
            }

            // 秩加1
            rank++;
        }

        // 矩阵可逆当且仅当秩等于阶数
        return rank == rows_;
    }

    // 将小矩阵嵌入到大矩阵的左上角（左上角对齐），返回新矩阵（原矩阵不变）
    Matrix leftEmbed(const Matrix& smallMat) const {  // 加const确保不修改原矩阵
        // 检查小矩阵是否超过大矩阵尺寸
        if (smallMat.rows() > rows_ || smallMat.cols() > cols_) {
            throw out_of_range("Small matrix is too large to embed in top-left corner!");
        }

        // 创建原矩阵的副本（新矩阵，初始值与原矩阵相同）
        Matrix result = *this;

        // 在副本的左上角嵌入小矩阵（覆盖对应位置）
        for (size_t i = 0; i < smallMat.rows(); ++i) {
            for (size_t j = 0; j < smallMat.cols(); ++j) {
                result(i, j) = smallMat(i, j);  // 修改副本，不影响原矩阵
            }
        }

        return result;  // 返回嵌入后的新矩阵
    }

    // 将小矩阵嵌入到大矩阵的右下角（右下角对齐），返回新矩阵（原矩阵不变）
    Matrix rightEmbed(const Matrix& smallMat) const {  // 加const确保不修改原矩阵
        // 计算右下角起始行和列
        size_t startRow = rows_ - smallMat.rows();
        size_t startCol = cols_ - smallMat.cols();

        // 检查小矩阵是否超过大矩阵尺寸
        if (smallMat.rows() > rows_ || smallMat.cols() > cols_) {
            throw out_of_range("Small matrix is too large to embed in bottom-right corner!");
        }

        // 创建原矩阵的副本（新矩阵，初始值与原矩阵相同）
        Matrix result = *this;

        // 在副本的右下角嵌入小矩阵（覆盖对应位置）
        for (size_t i = 0; i < smallMat.rows(); ++i) {
            for (size_t j = 0; j < smallMat.cols(); ++j) {
                result(startRow + i, startCol + j) = smallMat(i, j);  // 修改副本，不影响原矩阵
            }
        }

        return result;  // 返回嵌入后的新矩阵
    }
    // 返回动态数组（vector），直接包含全0行索引和大小
    vector<size_t> getZeroRowVector() const {
        vector<size_t> zeroRows;
        for (size_t row = 0; row < rows_; ++row) {
            if (isRowZero(row)) {
                zeroRows.push_back(row);  // 自动扩容
            }
        }
        return zeroRows;
    }

    // 全0列同理
    vector<size_t> getZeroColVector() const {
        vector<size_t> zeroCols;
        for (size_t col = 0; col < cols_; ++col) {
            if (isColumnZero(col)) {
                zeroCols.push_back(col);
            }
        }
        return zeroCols;
    }
    // 新增：查询指定行范围内的全0行（[startRow, endRow]，闭区间）
    vector<size_t> getZeroRowVectorInRange(size_t startRow, size_t endRow) const {
        vector<size_t> zeroRowsInRange;

        // 边界检查：确保startRow <= endRow，且不超过矩阵实际行数
        if (startRow > endRow || endRow >= rows_) {
            throw out_of_range("行范围超出矩阵有效行数！");
        }

        // 遍历指定范围内的行，检查是否为全0行
        for (size_t row = startRow; row <= endRow; ++row) {
            if (isRowZero(row)) {
                zeroRowsInRange.push_back(row);  // 存储原始行索引
            }
        }

        return zeroRowsInRange;
    }

    // 新增：查询指定列范围内的全0列（[startCol, endCol]，闭区间）
    vector<size_t> getZeroColVectorInRange(size_t startCol, size_t endCol) const {
        vector<size_t> zeroColsInRange;

        // 边界检查：确保startCol <= endCol，且不超过矩阵实际列数
        if (startCol > endCol || endCol >= cols_) {
            throw out_of_range("列范围超出矩阵有效列数！");
        }

        // 遍历指定范围内的列，检查是否为全0列
        for (size_t col = startCol; col <= endCol; ++col) {
            if (isColumnZero(col)) {
                zeroColsInRange.push_back(col);  // 存储原始列索引
            }
        }

        return zeroColsInRange;
    }
    // 删除指定行（参数：要删除的行索引vector，无unordered_set）
    Matrix removeRows(const vector<size_t>& rowIndices) const {
        if (rowIndices.empty()) return *this; // 无要删的行，返回原矩阵

        vector<size_t> keep; // 存储需要保留的行索引
        // 遍历原矩阵所有行，判断是否需要保留
        for (size_t i = 0; i < rows_; ++i) {
            bool needDelete = false;
            // 检查当前行是否在删除列表中
            for (size_t delRow : rowIndices) {
                if (i == delRow) {
                    needDelete = true;
                    break; // 找到就退出，不用继续遍历
                }
            }
            if (!needDelete) {
                keep.push_back(i); // 不需要删除，加入保留列表
            }
        }

        // 构造新矩阵
        Matrix res(keep.size(), cols_);
        for (size_t newRow = 0; newRow < keep.size(); ++newRow) {
            size_t oldRow = keep[newRow];
            for (size_t col = 0; col < cols_; ++col) {
                res(newRow, col) = (*this)(oldRow, col);
            }
        }
        return res;
    }

    // 删除指定列（参数：要删除的列索引vector，无unordered_set）
    Matrix removeCols(const vector<size_t>& colIndices) const {
        if (colIndices.empty()) return *this; // 无要删的列，返回原矩阵

        vector<size_t> keep; // 存储需要保留的列索引
        // 遍历原矩阵所有列，判断是否需要保留
        for (size_t j = 0; j < cols_; ++j) {
            bool needDelete = false;
            // 检查当前列是否在删除列表中
            for (size_t delCol : colIndices) {
                if (j == delCol) {
                    needDelete = true;
                    break; // 找到就退出，不用继续遍历
                }
            }
            if (!needDelete) {
                keep.push_back(j); // 不需要删除，加入保留列表
            }
        }

        // 构造新矩阵
        Matrix res(rows_, keep.size());
        for (size_t newCol = 0; newCol < keep.size(); ++newCol) {
            size_t oldCol = keep[newCol];
            for (size_t row = 0; row < rows_; ++row) {
                res(row, newCol) = (*this)(row, oldCol);
            }
        }
        return res;
    }
    // --------------------------
// 新增：removeRows 变长参数重载（支持传入单个/多个行索引，如 0、0,1,5）
// --------------------------
// 递归终止函数（无参数时，返回原矩阵）
    Matrix removeRows() const {
        return *this;
    }

    // 变长参数核心函数（接收1个索引 + 剩余参数，递归处理）
    template<typename... Args>
    Matrix removeRows(size_t firstRow, Args... restRows) const {
        // 1. 构造包含当前索引的 vector
        vector<size_t> rowIndices = { firstRow };
        // 2. 递归添加剩余索引（展开参数包）
        addArgsToVector(rowIndices, restRows...);
        // 3. 调用原有的 vector 版本函数完成删除
        return removeRows(rowIndices);
    }

    // --------------------------
    // 新增：removeCols 变长参数重载（支持传入单个/多个列索引，如 2、1,3,4）
    // --------------------------
    // 递归终止函数（无参数时，返回原矩阵）
    Matrix removeCols() const {
        return *this;
    }

    // 变长参数核心函数（接收1个索引 + 剩余参数，递归处理）
    template<typename... Args>
    Matrix removeCols(size_t firstCol, Args... restCols) const {
        // 1. 构造包含当前索引的 vector
        vector<size_t> colIndices = { firstCol };
        // 2. 递归添加剩余索引（展开参数包）
        addArgsToVector(colIndices, restCols...);
        // 3. 调用原有的 vector 版本函数完成删除
        return removeCols(colIndices);
    }
    // 在当前矩阵的行末尾追加一个行矩阵（返回新矩阵，原矩阵不变）
    // 要求：待追加矩阵必须是"行矩阵"（行数=1），且列数与当前矩阵相同，否则返回原矩阵副本
    Matrix appendRowMatrix(const Matrix& rowMat) const {
        // 复制原矩阵，作为操作的副本（保证原矩阵不变）
        Matrix newMat = *this;

        // 检查1：待追加矩阵是否为行矩阵（行数必须等于1）
        if (rowMat.rows() != 1) {
            cout << "[警告] 待追加的不是行矩阵（行数≠1），返回原矩阵！" << endl;
            return newMat;  // 返回原矩阵副本
        }

        // 检查2：待追加矩阵的列数是否与当前矩阵相同
        // 特殊情况：当前矩阵为空（行数=0或列数=0）时，直接以行矩阵的列数创建新矩阵
        if (newMat.rows() == 0 || newMat.cols() == 0) {
            // 新矩阵初始化为行矩阵的内容（1行N列）
            newMat.rows_ = 1;
            newMat.cols_ = rowMat.cols();
            newMat.data_.resize(newMat.rows_ * newMat.cols_);
            for (size_t j = 0; j < newMat.cols_; ++j) {
                newMat.data_[j] = rowMat(0, j);
            }
            return newMat;
        }

        // 当前矩阵非空，检查列数匹配
        if (rowMat.cols() != newMat.cols()) {
            cout << "[警告] 行矩阵列数（" << rowMat.cols() << "）与当前矩阵列数（"
                << newMat.cols() << "）不匹配，返回原矩阵！" << endl;
            return newMat;  // 列数不匹配，返回原矩阵副本
        }

        // 列数匹配，在新矩阵副本末尾追加行
        newMat.rows_ += 1;  // 行数+1
        newMat.data_.reserve(newMat.rows_ * newMat.cols_);  // 预留空间
        // 复制行矩阵的元素到新矩阵的最后一行
        for (size_t j = 0; j < newMat.cols_; ++j) {
            newMat.data_.push_back(rowMat(0, j));
        }

        return newMat;  // 返回追加后的新矩阵
    }
    // 列向量叉乘：当前矩阵（3×1列向量）与另一个3×1列向量的叉乘，返回3×1列向量结果
    Matrix cross(const Matrix& other) const {
        // 第一步：验证两个输入是否均为3×1的列向量
        if ((rows_ != 3 || cols_ != 1) || (other.rows_ != 3 || other.cols_ != 1)) {
            throw invalid_argument("叉乘仅支持两个3×1的列向量！");
        }

        // 第二步：提取当前列向量的三个元素，用于构造反对称矩阵
        double a1 = (*this)(0, 0);  // 当前向量第一行元素
        double a2 = (*this)(1, 0);  // 当前向量第二行元素
        double a3 = (*this)(2, 0);  // 当前向量第三行元素

        // 第三步：构造当前向量的3阶反对称矩阵 [this]_×
        Matrix antisymmetric_mat(3, 3, {
            {0,  -a3, a2},   // 第一行：0, -a3, a2
            {a3,  0, -a1},   // 第二行：a3, 0, -a1
            {-a2, a1, 0}     // 第三行：-a2, a1, 0
            });

        // 第四步：反对称矩阵与另一个列向量相乘，结果即为叉乘结果（3×1列向量）
        return antisymmetric_mat * other;
    }

    // 静态函数：通过两个3×1列向量计算叉乘（兼容非成员调用方式，可选）
    static Matrix cross(const Matrix& vec1, const Matrix& vec2) {
        // 直接调用成员函数，复用验证和计算逻辑
        return vec1.cross(vec2);
    }
    // 此处为原类的闭合大括号，确保函数在类内部

       // --------------------------
       // 辅助函数：将变长参数包中的索引添加到 vector（递归实现）
       // --------------------------
       // 递归终止函数（无剩余参数时，直接返回）
    void addArgsToVector(vector<size_t>&) const {
        return;
    }

    // 递归添加参数（接收1个索引 + 剩余参数，依次存入 vector）
    template<typename... Args>
    void addArgsToVector(vector<size_t>& vec, size_t arg, Args... restArgs) const {
        // 检查索引合法性（避免越界）
        if (vec.empty()) {
            // 若为列索引，检查列范围
            if (arg >= cols_) {
                throw out_of_range("列索引超出矩阵有效列数！");
            }
        }
        else {
            // 若为行索引，检查行范围
            if (arg >= rows_) {
                throw out_of_range("行索引超出矩阵有效行数！");
            }
        }
        // 添加当前索引到 vector
        vec.push_back(arg);
        // 递归处理剩余参数
        addArgsToVector(vec, restArgs...);
    }
    // 列向量求模：计算当前3×1列向量的模（L2范数），返回标量模值
    double norm() const {
        // 验证当前矩阵是否为3×1列向量（若需支持任意维度列向量，可删除rows_==3的判断）
        if (rows_ != 3 || cols_ != 1) {
            throw invalid_argument("求模函数仅支持3×1的列向量！");
        }

        // 提取列向量的三个元素
        double x = (*this)(0, 0);
        double y = (*this)(1, 0);
        double z = (*this)(2, 0);

        // 计算L2范数：sqrt(x? + y? + z?)，避免浮点精度问题（加1e-12防止开方结果为NaN）
        return sqrt(x * x + y * y + z * z + 1e-12);
    }
    // 列向量归一化：将当前3×1列向量除以自身模，返回单位列向量（模为1）
    Matrix unit() const {
        // 验证当前矩阵是否为3×1列向量
        if (rows_ != 3 || cols_ != 1) {
            throw invalid_argument("归一化函数仅支持3×1的列向量！");
        }

        // 调用已实现的norm()函数计算模值
        double n = norm();

        // 检查模是否为0（避免除以0错误）
        if (n < 1e-10) {
            throw runtime_error("无法对模为0的列向量进行归一化！");
        }

        // 创建归一化后的单位向量（每个元素除以模）
        Matrix unit_vec(3, 1);
        unit_vec(0, 0) = (*this)(0, 0) / n;
        unit_vec(1, 0) = (*this)(1, 0) / n;
        unit_vec(2, 0) = (*this)(2, 0) / n;

        return unit_vec;
    }
    // 单位向量叉乘：先将当前3×1列向量与另一个3×1列向量归一化（除以自身模），再计算叉乘
    // 返回值：归一化后两向量的叉乘结果（3×1列向量）
    Matrix unitCross(const Matrix& other) const {
        // 第一步：验证两个输入是否均为3×1列向量（复用叉乘的输入校验逻辑）
        if ((rows_ != 3 || cols_ != 1) || (other.rows_ != 3 || other.cols_ != 1)) {
            throw invalid_argument("单位向量叉乘仅支持两个3×1的列向量！");
        }

        // 第二步：分别计算两个向量的模（调用上面实现的norm()函数）
        double norm1 = this->norm();  // 当前向量的模
        double norm2 = other.norm();  // 另一个向量的模

        // 第三步：判断模是否为0（避免除以0错误，允许极小模值，用1e-10判断）
        if (norm1 < 1e-10 || norm2 < 1e-10) {
            throw runtime_error("无法对模为0的列向量进行归一化！");
        }

        // 第四步：将两个向量分别归一化（除以自身模，得到单位向量）
        Matrix vec1_unit(3, 1);  // 当前向量的单位向量
        vec1_unit(0, 0) = (*this)(0, 0) / norm1;
        vec1_unit(1, 0) = (*this)(1, 0) / norm1;
        vec1_unit(2, 0) = (*this)(2, 0) / norm1;

        Matrix vec2_unit(3, 1);  // 另一个向量的单位向量
        vec2_unit(0, 0) = other(0, 0) / norm2;
        vec2_unit(1, 0) = other(1, 0) / norm2;
        vec2_unit(2, 0) = other(2, 0) / norm2;

        // 第五步：调用已有的cross()函数，计算归一化后向量的叉乘
        return vec1_unit.cross(vec2_unit);
    }

    // 静态版单位向量叉乘：兼容非成员调用方式（直接传入两个3×1列向量）
    static Matrix unitCross(const Matrix& vec1, const Matrix& vec2) {
        // 复用成员函数逻辑，避免代码冗余
        return vec1.unitCross(vec2);
    }
    // 矩阵数乘（矩阵 × 数，成员函数，支持 mat * 5）
    Matrix operator*(double scalar) const {
        Matrix result(rows_, cols_);  // 创建与原矩阵同维度的结果矩阵
        for (size_t i = 0; i < rows_; ++i) {
            for (size_t j = 0; j < cols_; ++j) {
                result(i, j) = (*this)(i, j) * scalar;  // 每个元素乘以标量
            }
        }
        return result;
    }

    // 矩阵数乘（数 × 矩阵，友元函数，支持 5 * mat）
    friend Matrix operator*(double scalar, const Matrix& mat) {
        // 直接复用成员函数逻辑，避免代码冗余
        return mat * scalar;
    }

    // 矩阵数乘赋值（矩阵 ×= 数，直接修改原矩阵，支持 mat *= 5）
    Matrix& operator*=(double scalar) {
        for (size_t i = 0; i < rows_; ++i) {
            for (size_t j = 0; j < cols_; ++j) {
                (*this)(i, j) *= scalar;  // 每个元素直接乘以标量并覆盖原值
            }
        }
        return *this;  // 返回修改后的原矩阵引用
    }
    // 列向量转反对称矩阵（仅支持3×1列向量），返回新矩阵，不修改原矩阵
    Matrix SSM() const {
        // 第一步：校验输入是否为3×1的列向量
        if (rows_ != 3 || cols_ != 1) {
            throw invalid_argument("反对称矩阵仅支持3×1的列向量输入！");
        }

        // 第二步：提取列向量的三个元素
        double x = (*this)(0, 0);  // 第一行元素
        double y = (*this)(1, 0);  // 第二行元素
        double z = (*this)(2, 0);  // 第三行元素

        // 第三步：构造3×3反对称矩阵并返回（不修改原矩阵）
        return Matrix(3, 3, {
            {0,  -z,  y},   // 第一行：0, -z, y
            {z,   0, -x},   // 第二行：z, 0, -x
            {-y,  x,  0}    // 第三行：-y, x, 0
            });
    }

    // 静态重载版本（支持非成员调用，如 Matrix::skewSymmetric(vec)）
    static Matrix SSM(const Matrix& vec) {
        // 复用成员函数逻辑，避免代码冗余
        return vec.SSM();
    }

    static Matrix arrayToCol(const double arr[]) { // 静态成员函数
        if (arr == nullptr) throw invalid_argument("数组为空！");
        Matrix colMat(3, 1);
        colMat(0, 0) = arr[0];
        colMat(1, 0) = arr[1];
        colMat(2, 0) = arr[2];
        return colMat;
    }
    void fillBlock(size_t r0, size_t c0, const Matrix& B)
    {
        if (r0 + B.rows() > rows_ || c0 + B.cols() > cols_)
        {
            throw out_of_range("fillBlock 超出目标矩阵范围！");
        }

        for (size_t i = 0; i < B.rows(); ++i)
        {
            for (size_t j = 0; j < B.cols(); ++j)
            {
                (*this)(r0 + i, c0 + j) = B(i, j);
            }
        }
    }
    Matrix normalizeCnb() const
    {
        if (rows_ != 3 || cols_ != 3)
        {
            throw invalid_argument("normalizeCnb 仅支持 3x3 姿态矩阵！");
        }

        Matrix c1 = this->col(0);
        Matrix c2 = this->col(1);

        c1 = c1.unit();

        double proj12 = c1(0, 0) * c2(0, 0)
            + c1(1, 0) * c2(1, 0)
            + c1(2, 0) * c2(2, 0);

        c2 = c2 - c1 * proj12;
        c2 = c2.unit();

        Matrix c3 = c1.cross(c2);
        c3 = c3.unit();

        Matrix Cn(3, 3);
        Cn(0, 0) = c1(0, 0);  Cn(1, 0) = c1(1, 0);  Cn(2, 0) = c1(2, 0);
        Cn(0, 1) = c2(0, 0);  Cn(1, 1) = c2(1, 0);  Cn(2, 1) = c2(2, 0);
        Cn(0, 2) = c3(0, 0);  Cn(1, 2) = c3(1, 0);  Cn(2, 2) = c3(2, 0);

        return Cn;
    }
    // 输出矩阵（支持 cout << matrix）
    friend ostream& operator<<(ostream& os, const Matrix& mat) {
        for (size_t i = 0; i < mat.rows_; i++) {
            for (size_t j = 0; j < mat.cols_; j++) {
                os << mat(i, j) << " ";
            }
            os << "\n";
        }
        return os;
    }
};
