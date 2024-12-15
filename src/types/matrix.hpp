#ifndef HEADER_GUARD_MATRIX_HPP
#define HEADER_GUARD_MATRIX_HPP

#include <assert.h>

namespace fluid
{   
    template <typename T>
    class Matrix
    {
    public:
        Matrix() {};
        virtual T* operator[] (size_t index) = 0;
        virtual ~Matrix() = default;

        virtual size_t get_n() const = 0;
        virtual size_t get_m() const = 0;
    };

    template <typename T, size_t N, size_t M>
    class StaticMatrix : Matrix<T>
    {
    public:
        StaticMatrix(size_t n, size_t m) { assert(n == N && m == M); };
        StaticMatrix(const StaticMatrix& other);
        StaticMatrix& operator= (const StaticMatrix& other);

        ~StaticMatrix();

        T* operator[] (size_t index) override;

        size_t get_n() const override;
        size_t get_m() const override;

    private:
        T data[N][M];

        const size_t n = N;
        const size_t m = M;
    };


    template <typename T>
    class DynamicMatrix : Matrix<T>
    {
    public:
        DynamicMatrix(size_t n, size_t m);
        DynamicMatrix(const DynamicMatrix& other);
        DynamicMatrix& operator= (const DynamicMatrix& other);

        ~DynamicMatrix();

        T* operator[] (size_t index) override;

        size_t get_n() const override;
        size_t get_m() const override;

    private:
        void allocate();
        void deallocate();


        T** data;

        size_t n;
        size_t m;
    };

    // Static Matrix

    template <typename T, size_t N, size_t M>
    StaticMatrix<T, N, M>::StaticMatrix(const StaticMatrix<T, N, M>& other)
    {
        std::memcpy(this->data, other.data, sizeof(data));
    }

    template <typename T, size_t N, size_t M>
    StaticMatrix<T, N, M>& StaticMatrix<T, N, M>::operator= (const StaticMatrix<T, N, M>& other)
    {
        if (*this != &other)
            std::memcpy(this->data, other.data, sizeof(data));
        return *this;
    }

    template <typename T, size_t N, size_t M>
    T* StaticMatrix<T, N, M>::operator[] (size_t index)
    {
        assert(index < N);
        return data[index];
    }

    template <typename T, size_t N, size_t M>
    size_t StaticMatrix<T, N, M>::get_n() const { return n; }

    template <typename T, size_t N, size_t M>
    size_t StaticMatrix<T, N, M>::get_m() const { return m; }

    template <typename T, size_t N, size_t M>
    StaticMatrix<T, N, M>::~StaticMatrix() {}


    // Dynamic Matrix

    template <typename T>
    DynamicMatrix<T>::DynamicMatrix(size_t n, size_t m)
        : n(n), m(m)
    {
        allocate();
    }

    template <typename T>
    DynamicMatrix<T>::~DynamicMatrix()
    {
        deallocate();
    }

    template <typename T>
    void DynamicMatrix<T>::allocate()
    {
        data = new T*[n];
        for (auto i = 0LU; i < n; i++)
            data[i] = new T[m];
    }

    template <typename T>
    void DynamicMatrix<T>::deallocate()
    {
        for (auto i = 0LU; i < n; i++)
            delete[] data[i];
        delete[] data;
        data = nullptr;
        n = 0LU;
        m = 0LU;
    }

    template <typename T>
    DynamicMatrix<T>::DynamicMatrix(const DynamicMatrix<T>& other)
    : n(other.n), m(other.m)
    {
        allocate();
        for (auto i = 0LU; i < n; i++)
            std::memcpy(data[i], other.data[i], sizeof(T) * m);
    }

    template <typename T>
    DynamicMatrix<T>& DynamicMatrix<T>::operator= (const DynamicMatrix<T>& other)
    {
        if ((n != other.n) || (m != other.m))
        {
            deallocate();
            n = other.n;
            m = other.m;
            allocate();
        }

        for (auto i = 0LU; i < n; i++)
            std::memcpy(data[i], other.data[i], sizeof(T) * m);
    }

    template <typename T>
    T* DynamicMatrix<T>::operator[] (size_t index)
    {
        assert(index < n);
        return data[index];
    }

    template <typename T>
    size_t DynamicMatrix<T>::get_n() const { return n; }

    template <typename T>
    size_t DynamicMatrix<T>::get_m() const { return m; }

} // namespace fluid




#endif // HEADER_GUARD_MATRIX_HPP