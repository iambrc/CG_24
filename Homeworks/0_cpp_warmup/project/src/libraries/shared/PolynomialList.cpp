#include "PolynomialList.h"

#include <iostream>
#include <algorithm>
#include <fstream>
#include <cmath>

#define EPSILON 1.0e-10	// zero double
using namespace std;

PolynomialList::PolynomialList(const PolynomialList& other) 
{
    m_Polynomial = other.m_Polynomial;
}

PolynomialList::PolynomialList(const string& file) 
{
    ReadFromFile(file);
}

PolynomialList::PolynomialList(const double* cof, const int* deg, int n) 
{
    for (int i = 0;i < n; i++)
    {
        AddOneTerm(Term(deg[i],cof[i]));
    } 
}

PolynomialList::PolynomialList(const vector<int>& deg, const vector<double>& cof) 
{
    if (deg.size() != cof.size())
    {
        cout << "Size Error!" << endl;
    }
    else
    {
        for (size_t i = 0;i < deg.size();i++)
        {
            AddOneTerm(Term(deg[i],cof[i]));
        }
    }
}

double PolynomialList::coff(int i) const 
{
    for (const Term& term : m_Polynomial)
    {
        if (term.deg == i)
        {
            return term.cof;
        }
    }
    return 0.;
}

double& PolynomialList::coff(int i) 
{
    return AddOneTerm(Term(i, 0)).cof;
}

void PolynomialList::compress() 
{
    auto itr = m_Polynomial.begin();
    while (itr != m_Polynomial.end()) 
    {
        if (fabs((*itr).cof) < EPSILON)
        {
            itr = m_Polynomial.erase(itr);
        }
        else
        {
            itr++;
        }  
    }
}

PolynomialList PolynomialList::operator+(const PolynomialList& right) const 
{
    PolynomialList poly(*this);
    for (const auto& term : right.m_Polynomial)
    {
        poly.AddOneTerm(term);
    }
    poly.compress(); // delete terms with 0 coefficient
    return poly;
}

PolynomialList PolynomialList::operator-(const PolynomialList& right) const 
{
    PolynomialList poly(*this);
    for (const auto& term : right.m_Polynomial)
    {
        poly.AddOneTerm(Term(term.deg, -term.cof));
    }
    poly.compress(); // delete terms with 0 coefficient
    return poly;
}

PolynomialList PolynomialList::operator*(const PolynomialList& right) const 
{
    PolynomialList result;
    for (const auto& term1 : m_Polynomial)
    {
        for (const auto& term2 : right.m_Polynomial)
        {
            double new_cof = term1.cof * term2.cof;
            int new_deg = term1.deg + term2.deg;
            result.AddOneTerm(Term(new_deg, new_cof));
        }
    }
    result.compress();
    return result;
}

PolynomialList& PolynomialList::operator=(const PolynomialList& right) 
{
    m_Polynomial = right.m_Polynomial;
    return *this;
}

void PolynomialList::Print() const 
{
    auto itr = m_Polynomial.begin();
    if (itr == m_Polynomial.end())
    {
        cout << "0" << endl;
    }
    else
    {
        while (itr != m_Polynomial.end())
        {
            if (itr != m_Polynomial.begin())
            {
                cout << " ";
                if (itr->cof > 0)
                {
                    cout << "+";
                }
            }
            cout << itr->cof;
            if (itr->deg > 0)
            {
                cout << "x^" << itr->deg;
            }
            itr++;
        }
    }
    cout << endl;
}

bool PolynomialList::ReadFromFile(const string& file) 
{
    m_Polynomial.clear();

    ifstream inp;
    inp.open(file.c_str());
    if (!inp.is_open()) {
        cout << "ERROR::PolynomialList::ReadFromFile:" << endl
            << "\t" << "file [" << file << "] opens failed" << endl;
        return false;
    }

    char ch;
    int n;
    inp >> ch;
    inp >> n;
    for (int i = 0; i < n; i++) {
        Term nd;
        inp >> nd.deg;
        inp >> nd.cof;

        AddOneTerm(nd);
    }

    inp.close();

    return true;
}

PolynomialList::Term& PolynomialList::AddOneTerm(const Term& term) 
{
    auto itr = m_Polynomial.begin();

    while (itr != m_Polynomial.end())
    {
        if (itr->deg == term.deg)
        {
            itr->cof += term.cof;
            return *itr;
        }
        itr++;
    }
    return *m_Polynomial.insert(itr, term);
}
