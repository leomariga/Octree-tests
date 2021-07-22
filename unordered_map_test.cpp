// Example program
#include <iostream>
#include <string>

#include <unordered_map>
#include <cstdint>

struct Foo
{
    float x;
    float y;
    float z;
};

// https://stackoverflow.com/questions/25375202/how-to-measure-memory-usage-of-stdunordered-map
int get_mem(std::unordered_map<uint32_t, Foo> mymap){
    int size_elements = mymap.size();
    std::cout << "sizeof mymap: " << sizeof(mymap) << std::endl;
    std::cout << "number of elements: " << mymap.size() << std::endl;
    std::cout << "number of buckets: " << mymap.bucket_count() << std::endl;
    
    std::cout << "size of element list: " << mymap.size() * 3*8 << std::endl;
    std::cout << "pointers to next element" << mymap.size() * sizeof(void*) << std::endl;
    std::cout << sizeof(void*)<< std::endl;
    std::cout << "sizeof hash table buckets: " << mymap.bucket_count() * (sizeof(size_t) + sizeof(void*)) << std::endl;
    std::cout << (sizeof(size_t) + sizeof(void*)) << std::endl;

    return sizeof(mymap);
}

int32_t main(int32_t argc, char** argv)
{
    std::unordered_map<uint32_t, Foo> mapNoReserve;
    std::unordered_map<uint32_t, Foo> mapReserve;
    
    
    

    // --> Snapshot A

    mapReserve.reserve(1010);
    
    std::cout << "teste" << get_mem(mapReserve)<< std::endl;

    // --> Snapshot B

    for(uint32_t i = 0; i < 100; ++i)
    {
        mapNoReserve.insert(std::make_pair(i, Foo()));
        mapReserve.insert(std::make_pair(i, Foo()));
    }

    std::cout << "teste" << get_mem(mapReserve)<< std::endl;
    // -> Snapshot C

    return 0;
}