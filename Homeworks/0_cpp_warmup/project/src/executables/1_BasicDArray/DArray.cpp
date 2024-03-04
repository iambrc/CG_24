// implementation of class DArray
#include "DArray.h"
#include <cstdio>
#include <cstdlib>

// default constructor
DArray::DArray() 
{
	Init();
}

// set an array with default values
DArray::DArray(int nSize, double dValue) 
{
	m_nSize = nSize;
	m_pData = new double[nSize];
	for (int i = 0;i < nSize;i++)
	{
		m_pData[i] = dValue;
	}
}

DArray::DArray(const DArray& arr) 
{
	m_nSize = arr.GetSize();
	m_pData = new double[m_nSize];
	for (int i = 0;i < m_nSize;i++)
	{
		m_pData[i] = arr.GetAt(i);
	}
}

// deconstructor
DArray::~DArray() 
{
	Free();
}

// display the elements of the array
void DArray::Print() const 
{
	for (int i = 0;i < m_nSize;i++)
	{
		printf("%lf ", m_pData[i]);
	}
	printf("\n");
}

// initilize the array
void DArray::Init() 
{
	m_nSize = 0;
	m_pData = NULL;
}

// free the array
void DArray::Free() 
{
	delete[] m_pData;
	m_pData = NULL;
	m_nSize = 0;
}

// get the size of the array
int DArray::GetSize() const 
{
	return m_nSize;
}

// set the size of the array
void DArray::SetSize(int nSize) 
{
	if (m_nSize == nSize)
	{
		return;
	}
	double* tmp = new double[nSize];  //此处需深拷贝
	if (m_nSize < nSize)
	{
		for (int i = 0;i < m_nSize;i++)
		{
			tmp[i] = m_pData[i];
		}
		for (int i = m_nSize;i < nSize;i++)
		{
			tmp[i] = 0.;
		}
	}
	else
	{
		for (int i = 0;i < nSize;i++)
		{
			tmp[i] = m_pData[i];
		}
	}
	m_nSize = nSize;
	delete[] m_pData;
	m_pData = tmp;
}

// get an element at an index
const double& DArray::GetAt(int nIndex) const 
{
	if (nIndex >= 0 && nIndex < m_nSize)
	{
		return m_pData[nIndex];
	}
	else
	{
		printf("Index Error!\n");
		return 0;
	}
}

// set the value of an element 
void DArray::SetAt(int nIndex, double dValue) 
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
const double& DArray::operator[](int nIndex) const 
{
	return m_pData[nIndex];
}

double& DArray::operator[](int nIndex)
{
	return m_pData[nIndex];
}

// add a new element at the end of the array
void DArray::PushBack(double dValue) 
{
	SetSize(m_nSize + 1);
	m_pData[m_nSize - 1] = dValue;
}

// delete an element at some index
void DArray::DeleteAt(int nIndex) 
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
void DArray::InsertAt(int nIndex, double dValue) 
{
	if (nIndex >= 0 && nIndex <= m_nSize)
	{
		m_nSize++;
		double* tmp = new double[m_nSize];
		for (int i = 0;i < nIndex;i++)
		{
			tmp[i] = m_pData[i];
		}
		tmp[nIndex] = dValue;
		for (int i = nIndex + 1;i < m_nSize;i++)
		{
			tmp[i] = m_pData[i - 1];
		}
		delete[] m_pData;
		m_pData = tmp;
	}
	else
	{
		printf("Index Error!\n");
	}
}

// overload operator '='
DArray& DArray::operator = (const DArray& arr) 
{
	if (this != &arr)
	{
		delete[]  m_pData;
		m_nSize = arr.GetSize();
		m_pData = new double[m_nSize];
		for (int i = 0;i < m_nSize;i++)
		{
			m_pData[i] = arr.GetAt(i);
		}
	}
	return *this;
}
