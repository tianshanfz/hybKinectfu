#ifndef WEIGHTDATACALCULATER_H
#define WEIGHTDATACALCULATER_H
#include<vector>
using namespace std;
template <typename T>
struct WeightDataStruct{//must be weight>=0
    T data;
    float weight;
    WeightDataStruct(const T& d,float w):data(d),weight(w){}
};

template <typename T>
class WeightDataCalculater
{
public:
    WeightDataCalculater();
    virtual ~WeightDataCalculater();

    static T calWeightMedian(const vector<T> &datas,const vector<float>& weights)
    {

        vector<WeightDataStruct<T>> inputs;
        int n=datas.size();
        float sum_weight=0;
        for(int i=0;i<n;i++)
        {
            sum_weight+=weights[i];
            inputs.push_back({datas[i],weights[i]});
        }
        sort(inputs.begin(),inputs.end(),[](const WeightDataStruct<T>& i,const WeightDataStruct<T>& j){return i.data<j.data;});
        return calWeightMedianInternal(inputs,sum_weight,0,n-1);
    }

protected:

private:
    static T calWeightMedianInternal( vector<WeightDataStruct<T>>& inputs,float sum_weights,int l,int r)
    {//must be inputs sorted and 0<=l<=r<n
        if(r-l+1==1)return inputs[l].data;
        if(r-l+1==2)return inputs[l].weight>sum_weights/2?inputs[l].data:inputs[r].data;
        int mid=(l+r)/2;
        float left_weights=0;
        for_each(inputs.begin()+l,inputs.begin()+mid,[&](const WeightDataStruct<T>& p){left_weights+=p.weight;});
        float right_weights=sum_weights-left_weights-inputs[mid].weight;
        if(left_weights<sum_weights/2&&right_weights<sum_weights/2)return inputs[mid].data;
        else if(left_weights>=sum_weights/2)
        {
            inputs[mid].weight+=right_weights;
            return calWeightMedianInternal(inputs,sum_weights,l,mid);
        }
        else
        {
            inputs[mid].weight+=left_weights;
            return calWeightMedianInternal(inputs,sum_weights,mid,r);
        }
    }
};

#endif // WEIGHTDATACALCULATER_H
