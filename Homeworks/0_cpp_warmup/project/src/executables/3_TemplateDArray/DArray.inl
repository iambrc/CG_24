#include "DArray.h"
#include <iostream>

using namespace std;

// default constructor
template<typename T>
DArray<T>::DArray() 
{
	Init();
}

// set an array with default values
template<typename T>
DArray<T>::DArray(int nSize, const T& dValue) 
{
	m_nSize = nSize;
	m_nMax =  nSize;
	m_pData = new double[nSize];
	for (int i = 0;i < nSize;i++)
	{
		m_pData[i] = dValue;
	}
}

template<typename T>
DArray<T>::DArray(const DArray& arr) 
{
	m_nSize = arr.GetSize();
	m_nMax = m_nSize;
	m_pData = new double[m_nSize];
	for (int i = 0;i < m_nSize;i++)
	{
		m_pData[i] = arr.GetAt(i);
	}
}

// deconstructor
template<typename T>
DArray<T>::~DArray() 
{
	Free();
}

// display the elements of the array
template<typename T>
void DArray<T>::Print() const 
{
	for (int i = 0;i < m_nSize;i++)
	{
		cout << m_pData[i] << " ";
	}
	cout << endl;
}

// initilize the array
template<typename T>
void DArray<T>::Init() 
{
	m_nSize = 0;
	m_nMax = 0;
	m_pData = NULL;
}

// free the array
template<typename T>
void DArray<T>::Free() 
{
	delete[] m_pData;
	m_pData = NULL;
	m_nSize = 0;
}

template<typename T>
void DArray<T>::Reserve(int nSize)
{
	if (m_nMax >= nSize)
	{
		return;
	}
	while (m_nMax < nSize)
	{
		if (m_nMax == 0)
		{
			m_nMax = 1;
		}
		else
		{
			m_nMax = 2 * m_nMax;
		}
	}
	T* pData = new T[m_nMax];
	for(int i = 0;i < m_nSize;i++)
	{
		pData[i] = m_pData[i];
	}
	delete[] m_pData;
	m_pData = pData;
}

// get the size of the array
template<typename T>
int DArray<T>::GetSize() const 
{
	return m_nSize;
}

// set the size of the array
template<typename T>
void DArray<T>::SetSize(int nSize) 
{
	if (m_nSize == nSize)
	{
		return;
	}
	Reserve(nSize);
	for (int i = m_nSize;i < nSize;i++)
	{
		m_pData[i] = 0.;
	}
	m_nSize = nSize;
}

// get an element at an index
template<typename T>
const T& DArray<T>::GetAt(int nIndex) const 
{
	if (nIndex >= 0 && nIndex < m_nSize)
	{
		return m_pData[nIndex];
	}
	else
	{
		printf("Index Error!\n");
		return m_pData[0];
	}
}

// set the value of an element 
template<typename T>
void DArray<T>::SetAt(int nIndex, const T& dValue) 
{
	if (nIndex >= 0 && nIndex < m_nSize)
	{
		m_pData[nIndex] = dValue;
	}
	else
	{
		printf("Index Error!\n");
	}
}

// overload operator '[]'
template<typename T>
const T& DArray<T>::operator[](int nIndex) const 
{
	return m_pData[nIndex];
}

template<typename T>
T& DArray<T>::operator[](int nIndex)
{
	return m_pData[nIndex];
}

// add a new element at the end of the array
template<typename T>
void DArray<T>::PushBack(const T& dValue) 
{
	SetSize(m_nSize + 1);
	m_pData[m_nSize - 1] = dValue;
}

// delete an element at some index
template<typename T>
void DArray<T>::DeleteAt(int nIndex) 
{
	if (nIndex >= 0 && nIndex < m_nSize)
	{
		for (int i = nIndex + 1; i < m_nSize; i++)
		{
			m_pData[i - 1] = m_pData[i];
		}
		m_nSize--;
	}
	else
	{
		printf("Index Error!\n");
	}
}

// insert a new element at some index
template<typename T>
void DArray<T>::InsertAt(int nIndex, const T& dValue) 
{
	if (nIndex >= 0 && nIndex <= m_nSize)
	{
		Reserve(m_nSize + 1);

		for (int i = m_nSize; i > nIndex; i--)
		{
			m_pData[i] = m_pData[i - 1];
		}
		m_pData[nIndex] = dValue;
		m_nSize++;
	}
	else
	{
		printf("Index Error!\n");
	}
}

// overload operator '='
template<typename T>
DArray<T>& DArray<T>::operator = (const DArray& arr) 
{
	if (this != &arr)
	{
		delete[]  m_pData;
		m_nSize = arr.GetSize();
		m_pData = new T[m_nSize];
		for (int i = 0;i < m_nSize;i++)
		{
			m_pData[i] = arr.GetAt(i);
		}
	}
	return *this;
}
