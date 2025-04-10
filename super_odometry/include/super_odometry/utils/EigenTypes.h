#pragma once

#include <deque>
#include <map>
#include <unordered_map>
#include <vector>
#include <list>
#include <Eigen/Core>
#include <Eigen/Dense>
// Eigen typedefs matrices and vectors

// Eigen typedefs matrices and vectors
namespace Eigen
{
template<typename T>
using aligned_vector = std::vector<T, Eigen::aligned_allocator<T>>;
template<typename T>
using aligned_list = std::list<T, Eigen::aligned_allocator<T>>;
template<typename T>
using aligned_deque = std::deque<T, Eigen::aligned_allocator<T>>;

template<typename K, typename V>
using aligned_map = std::map<K, V, std::less<K>,
                             Eigen::aligned_allocator<std::pair<K const, V>>>;

template<typename K, typename V>
using aligned_unordered_map =
std::unordered_map<K, V, std::hash<K>, std::equal_to<K>,
                   Eigen::aligned_allocator<std::pair<K const, V>>>;

} //namespace Eigen
namespace Eigen
{

// unsigned char matrices
typedef Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> MatXXu;

typedef Eigen::Matrix<unsigned char, 10, 10> Mat1010u;		//10
typedef Eigen::Matrix<unsigned char, 10, 9> Mat109u;
typedef Eigen::Matrix<unsigned char, 10, 8> Mat108u;
typedef Eigen::Matrix<unsigned char, 10, 7> Mat107u;
typedef Eigen::Matrix<unsigned char, 10, 6> Mat106u;
typedef Eigen::Matrix<unsigned char, 10, 5> Mat105u;
typedef Eigen::Matrix<unsigned char, 10, 4> Mat104u;
typedef Eigen::Matrix<unsigned char, 10, 3> Mat103u;
typedef Eigen::Matrix<unsigned char, 10, 2> Mat102u;
typedef Eigen::Matrix<unsigned char, 2, 10> Mat210u;
typedef Eigen::Matrix<unsigned char, 3, 10> Mat310u;
typedef Eigen::Matrix<unsigned char, 4, 10> Mat410u;
typedef Eigen::Matrix<unsigned char, 5, 10> Mat510u;
typedef Eigen::Matrix<unsigned char, 6, 10> Mat610u;
typedef Eigen::Matrix<unsigned char, 7, 10> Mat710u;
typedef Eigen::Matrix<unsigned char, 8, 10> Mat810u;
typedef Eigen::Matrix<unsigned char, 9, 10> Mat910u;

typedef Eigen::Matrix<unsigned char, 9, 9> Mat99u;		//9
typedef Eigen::Matrix<unsigned char, 9, 8> Mat98u;
typedef Eigen::Matrix<unsigned char, 9, 7> Mat97u;
typedef Eigen::Matrix<unsigned char, 9, 6> Mat96u;
typedef Eigen::Matrix<unsigned char, 9, 5> Mat95u;
typedef Eigen::Matrix<unsigned char, 9, 4> Mat94u;
typedef Eigen::Matrix<unsigned char, 9, 3> Mat93u;
typedef Eigen::Matrix<unsigned char, 9, 2> Mat92u;
typedef Eigen::Matrix<unsigned char, 2, 9> Mat29u;
typedef Eigen::Matrix<unsigned char, 3, 9> Mat39u;
typedef Eigen::Matrix<unsigned char, 4, 9> Mat49u;
typedef Eigen::Matrix<unsigned char, 5, 9> Mat59u;
typedef Eigen::Matrix<unsigned char, 6, 9> Mat69u;
typedef Eigen::Matrix<unsigned char, 7, 9> Mat79u;
typedef Eigen::Matrix<unsigned char, 8, 9> Mat89u;

typedef Eigen::Matrix<unsigned char, 8, 8> Mat88u;		//8
typedef Eigen::Matrix<unsigned char, 8, 7> Mat87u;
typedef Eigen::Matrix<unsigned char, 8, 6> Mat86u;
typedef Eigen::Matrix<unsigned char, 8, 5> Mat85u;
typedef Eigen::Matrix<unsigned char, 8, 4> Mat84u;
typedef Eigen::Matrix<unsigned char, 8, 3> Mat83u;
typedef Eigen::Matrix<unsigned char, 8, 2> Mat82u;
typedef Eigen::Matrix<unsigned char, 2, 8> Mat28u;
typedef Eigen::Matrix<unsigned char, 3, 8> Mat38u;
typedef Eigen::Matrix<unsigned char, 4, 8> Mat48u;
typedef Eigen::Matrix<unsigned char, 5, 8> Mat58u;
typedef Eigen::Matrix<unsigned char, 6, 8> Mat68u;
typedef Eigen::Matrix<unsigned char, 7, 8> Mat78u;

typedef Eigen::Matrix<unsigned char, 7, 7> Mat77u;		//7
typedef Eigen::Matrix<unsigned char, 7, 6> Mat76u;
typedef Eigen::Matrix<unsigned char, 7, 5> Mat75u;
typedef Eigen::Matrix<unsigned char, 7, 4> Mat74u;
typedef Eigen::Matrix<unsigned char, 7, 3> Mat73u;
typedef Eigen::Matrix<unsigned char, 7, 2> Mat72u;
typedef Eigen::Matrix<unsigned char, 2, 7> Mat27u;
typedef Eigen::Matrix<unsigned char, 3, 7> Mat37u;
typedef Eigen::Matrix<unsigned char, 4, 7> Mat47u;
typedef Eigen::Matrix<unsigned char, 5, 7> Mat57u;
typedef Eigen::Matrix<unsigned char, 6, 7> Mat67u;

typedef Eigen::Matrix<unsigned char, 6, 6> Mat66u;		//6
typedef Eigen::Matrix<unsigned char, 6, 5> Mat65u;
typedef Eigen::Matrix<unsigned char, 6, 4> Mat64u;
typedef Eigen::Matrix<unsigned char, 6, 3> Mat63u;
typedef Eigen::Matrix<unsigned char, 6, 2> Mat62u;
typedef Eigen::Matrix<unsigned char, 2, 6> Mat26u;
typedef Eigen::Matrix<unsigned char, 3, 6> Mat36u;
typedef Eigen::Matrix<unsigned char, 4, 6> Mat46u;
typedef Eigen::Matrix<unsigned char, 5, 6> Mat56u;

typedef Eigen::Matrix<unsigned char, 5, 5> Mat55u;		//5
typedef Eigen::Matrix<unsigned char, 5, 4> Mat54u;
typedef Eigen::Matrix<unsigned char, 5, 3> Mat53u;
typedef Eigen::Matrix<unsigned char, 5, 2> Mat52u;
typedef Eigen::Matrix<unsigned char, 2, 5> Mat25u;
typedef Eigen::Matrix<unsigned char, 3, 5> Mat35u;
typedef Eigen::Matrix<unsigned char, 4, 5> Mat45u;

typedef Eigen::Matrix<unsigned char, 4, 4> Mat44u;		//4
typedef Eigen::Matrix<unsigned char, 4, 3> Mat43u;
typedef Eigen::Matrix<unsigned char, 4, 2> Mat42u;
typedef Eigen::Matrix<unsigned char, 2, 4> Mat24u;
typedef Eigen::Matrix<unsigned char, 3, 4> Mat34u;
typedef Eigen::Matrix<unsigned char, 4, 4> Mat44u;

typedef Eigen::Matrix<unsigned char, 3, 3> Mat33u;		//3
typedef Eigen::Matrix<unsigned char, 3, 2> Mat32u;
typedef Eigen::Matrix<unsigned char, 2, 3> Mat23u;

typedef Eigen::Matrix<unsigned char, 2, 2> Mat22u;		//2

// unsigned char vectors
typedef Eigen::Matrix<unsigned char, Eigen::Dynamic, 1> VecXu;

typedef Eigen::Matrix<unsigned char, 10, 1> Vec10u;
typedef Eigen::Matrix<unsigned char, 9, 1> Vec9u;
typedef Eigen::Matrix<unsigned char, 8, 1> Vec8u;
typedef Eigen::Matrix<unsigned char, 7, 1> Vec7u;
typedef Eigen::Matrix<unsigned char, 6, 1> Vec6u;
typedef Eigen::Matrix<unsigned char, 5, 1> Vec5u;
typedef Eigen::Matrix<unsigned char, 4, 1> Vec4u;
typedef Eigen::Matrix<unsigned char, 3, 1> Vec3u;
typedef Eigen::Matrix<unsigned char, 2, 1> Vec2u;

// unsigned int matrices
typedef Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic> MatXXui;

typedef Eigen::Matrix<unsigned int, 10, 10> Mat1010ui;		//10
typedef Eigen::Matrix<unsigned int, 10, 9> Mat109ui;
typedef Eigen::Matrix<unsigned int, 10, 8> Mat108ui;
typedef Eigen::Matrix<unsigned int, 10, 7> Mat107ui;
typedef Eigen::Matrix<unsigned int, 10, 6> Mat106ui;
typedef Eigen::Matrix<unsigned int, 10, 5> Mat105ui;
typedef Eigen::Matrix<unsigned int, 10, 4> Mat104ui;
typedef Eigen::Matrix<unsigned int, 10, 3> Mat103ui;
typedef Eigen::Matrix<unsigned int, 10, 2> Mat102ui;
typedef Eigen::Matrix<unsigned int, 2, 10> Mat210ui;
typedef Eigen::Matrix<unsigned int, 3, 10> Mat310ui;
typedef Eigen::Matrix<unsigned int, 4, 10> Mat410ui;
typedef Eigen::Matrix<unsigned int, 5, 10> Mat510ui;
typedef Eigen::Matrix<unsigned int, 6, 10> Mat610ui;
typedef Eigen::Matrix<unsigned int, 7, 10> Mat710ui;
typedef Eigen::Matrix<unsigned int, 8, 10> Mat810ui;
typedef Eigen::Matrix<unsigned int, 9, 10> Mat910ui;

typedef Eigen::Matrix<unsigned int, 9, 9> Mat99ui;		//9
typedef Eigen::Matrix<unsigned int, 9, 8> Mat98ui;
typedef Eigen::Matrix<unsigned int, 9, 7> Mat97ui;
typedef Eigen::Matrix<unsigned int, 9, 6> Mat96ui;
typedef Eigen::Matrix<unsigned int, 9, 5> Mat95ui;
typedef Eigen::Matrix<unsigned int, 9, 4> Mat94ui;
typedef Eigen::Matrix<unsigned int, 9, 3> Mat93ui;
typedef Eigen::Matrix<unsigned int, 9, 2> Mat92ui;
typedef Eigen::Matrix<unsigned int, 2, 9> Mat29ui;
typedef Eigen::Matrix<unsigned int, 3, 9> Mat39ui;
typedef Eigen::Matrix<unsigned int, 4, 9> Mat49ui;
typedef Eigen::Matrix<unsigned int, 5, 9> Mat59ui;
typedef Eigen::Matrix<unsigned int, 6, 9> Mat69ui;
typedef Eigen::Matrix<unsigned int, 7, 9> Mat79ui;
typedef Eigen::Matrix<unsigned int, 8, 9> Mat89ui;

typedef Eigen::Matrix<unsigned int, 8, 8> Mat88ui;		//8
typedef Eigen::Matrix<unsigned int, 8, 7> Mat87ui;
typedef Eigen::Matrix<unsigned int, 8, 6> Mat86ui;
typedef Eigen::Matrix<unsigned int, 8, 5> Mat85ui;
typedef Eigen::Matrix<unsigned int, 8, 4> Mat84ui;
typedef Eigen::Matrix<unsigned int, 8, 3> Mat83ui;
typedef Eigen::Matrix<unsigned int, 8, 2> Mat82ui;
typedef Eigen::Matrix<unsigned int, 2, 8> Mat28ui;
typedef Eigen::Matrix<unsigned int, 3, 8> Mat38ui;
typedef Eigen::Matrix<unsigned int, 4, 8> Mat48ui;
typedef Eigen::Matrix<unsigned int, 5, 8> Mat58ui;
typedef Eigen::Matrix<unsigned int, 6, 8> Mat68ui;
typedef Eigen::Matrix<unsigned int, 7, 8> Mat78ui;

typedef Eigen::Matrix<unsigned int, 7, 7> Mat77ui;		//7
typedef Eigen::Matrix<unsigned int, 7, 6> Mat76ui;
typedef Eigen::Matrix<unsigned int, 7, 5> Mat75ui;
typedef Eigen::Matrix<unsigned int, 7, 4> Mat74ui;
typedef Eigen::Matrix<unsigned int, 7, 3> Mat73ui;
typedef Eigen::Matrix<unsigned int, 7, 2> Mat72ui;
typedef Eigen::Matrix<unsigned int, 2, 7> Mat27ui;
typedef Eigen::Matrix<unsigned int, 3, 7> Mat37ui;
typedef Eigen::Matrix<unsigned int, 4, 7> Mat47ui;
typedef Eigen::Matrix<unsigned int, 5, 7> Mat57ui;
typedef Eigen::Matrix<unsigned int, 6, 7> Mat67ui;

typedef Eigen::Matrix<unsigned int, 6, 6> Mat66ui;		//6
typedef Eigen::Matrix<unsigned int, 6, 5> Mat65ui;
typedef Eigen::Matrix<unsigned int, 6, 4> Mat64ui;
typedef Eigen::Matrix<unsigned int, 6, 3> Mat63ui;
typedef Eigen::Matrix<unsigned int, 6, 2> Mat62ui;
typedef Eigen::Matrix<unsigned int, 2, 6> Mat26ui;
typedef Eigen::Matrix<unsigned int, 3, 6> Mat36ui;
typedef Eigen::Matrix<unsigned int, 4, 6> Mat46ui;
typedef Eigen::Matrix<unsigned int, 5, 6> Mat56ui;

typedef Eigen::Matrix<unsigned int, 5, 5> Mat55ui;		//5
typedef Eigen::Matrix<unsigned int, 5, 4> Mat54ui;
typedef Eigen::Matrix<unsigned int, 5, 3> Mat53ui;
typedef Eigen::Matrix<unsigned int, 5, 2> Mat52ui;
typedef Eigen::Matrix<unsigned int, 2, 5> Mat25ui;
typedef Eigen::Matrix<unsigned int, 3, 5> Mat35ui;
typedef Eigen::Matrix<unsigned int, 4, 5> Mat45ui;

typedef Eigen::Matrix<unsigned int, 4, 4> Mat44ui;		//4
typedef Eigen::Matrix<unsigned int, 4, 3> Mat43ui;
typedef Eigen::Matrix<unsigned int, 4, 2> Mat42ui;
typedef Eigen::Matrix<unsigned int, 2, 4> Mat24ui;
typedef Eigen::Matrix<unsigned int, 3, 4> Mat34ui;
typedef Eigen::Matrix<unsigned int, 4, 4> Mat44ui;

typedef Eigen::Matrix<unsigned int, 3, 3> Mat33ui;		//3
typedef Eigen::Matrix<unsigned int, 3, 2> Mat32ui;
typedef Eigen::Matrix<unsigned int, 2, 3> Mat23ui;

typedef Eigen::Matrix<unsigned int, 2, 2> Mat22ui;		//2

// unsigned int vectors
typedef Eigen::Matrix<unsigned int, Eigen::Dynamic, 1> VecXui;

typedef Eigen::Matrix<unsigned int, 10, 1> Vec10ui;
typedef Eigen::Matrix<unsigned int, 9, 1> Vec9ui;
typedef Eigen::Matrix<unsigned int, 8, 1> Vec8ui;
typedef Eigen::Matrix<unsigned int, 7, 1> Vec7ui;
typedef Eigen::Matrix<unsigned int, 6, 1> Vec6ui;
typedef Eigen::Matrix<unsigned int, 5, 1> Vec5ui;
typedef Eigen::Matrix<unsigned int, 4, 1> Vec4ui;
typedef Eigen::Matrix<unsigned int, 3, 1> Vec3ui;
typedef Eigen::Matrix<unsigned int, 2, 1> Vec2ui;

// int matrices
typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> MatXXi;

typedef Eigen::Matrix<int, 10, 10> Mat1010i;		//10
typedef Eigen::Matrix<int, 10, 9> Mat109i;
typedef Eigen::Matrix<int, 10, 8> Mat108i;
typedef Eigen::Matrix<int, 10, 7> Mat107i;
typedef Eigen::Matrix<int, 10, 6> Mat106i;
typedef Eigen::Matrix<int, 10, 5> Mat105i;
typedef Eigen::Matrix<int, 10, 4> Mat104i;
typedef Eigen::Matrix<int, 10, 3> Mat103i;
typedef Eigen::Matrix<int, 10, 2> Mat102i;
typedef Eigen::Matrix<int, 2, 10> Mat210i;
typedef Eigen::Matrix<int, 3, 10> Mat310i;
typedef Eigen::Matrix<int, 4, 10> Mat410i;
typedef Eigen::Matrix<int, 5, 10> Mat510i;
typedef Eigen::Matrix<int, 6, 10> Mat610i;
typedef Eigen::Matrix<int, 7, 10> Mat710i;
typedef Eigen::Matrix<int, 8, 10> Mat810i;
typedef Eigen::Matrix<int, 9, 10> Mat910i;

typedef Eigen::Matrix<int, 9, 9> Mat99i;		//9
typedef Eigen::Matrix<int, 9, 8> Mat98i;
typedef Eigen::Matrix<int, 9, 7> Mat97i;
typedef Eigen::Matrix<int, 9, 6> Mat96i;
typedef Eigen::Matrix<int, 9, 5> Mat95i;
typedef Eigen::Matrix<int, 9, 4> Mat94i;
typedef Eigen::Matrix<int, 9, 3> Mat93i;
typedef Eigen::Matrix<int, 9, 2> Mat92i;
typedef Eigen::Matrix<int, 2, 9> Mat29i;
typedef Eigen::Matrix<int, 3, 9> Mat39i;
typedef Eigen::Matrix<int, 4, 9> Mat49i;
typedef Eigen::Matrix<int, 5, 9> Mat59i;
typedef Eigen::Matrix<int, 6, 9> Mat69i;
typedef Eigen::Matrix<int, 7, 9> Mat79i;
typedef Eigen::Matrix<int, 8, 9> Mat89i;

typedef Eigen::Matrix<int, 8, 8> Mat88i;		//8
typedef Eigen::Matrix<int, 8, 7> Mat87i;
typedef Eigen::Matrix<int, 8, 6> Mat86i;
typedef Eigen::Matrix<int, 8, 5> Mat85i;
typedef Eigen::Matrix<int, 8, 4> Mat84i;
typedef Eigen::Matrix<int, 8, 3> Mat83i;
typedef Eigen::Matrix<int, 8, 2> Mat82i;
typedef Eigen::Matrix<int, 2, 8> Mat28i;
typedef Eigen::Matrix<int, 3, 8> Mat38i;
typedef Eigen::Matrix<int, 4, 8> Mat48i;
typedef Eigen::Matrix<int, 5, 8> Mat58i;
typedef Eigen::Matrix<int, 6, 8> Mat68i;
typedef Eigen::Matrix<int, 7, 8> Mat78i;

typedef Eigen::Matrix<int, 7, 7> Mat77i;		//7
typedef Eigen::Matrix<int, 7, 6> Mat76i;
typedef Eigen::Matrix<int, 7, 5> Mat75i;
typedef Eigen::Matrix<int, 7, 4> Mat74i;
typedef Eigen::Matrix<int, 7, 3> Mat73i;
typedef Eigen::Matrix<int, 7, 2> Mat72i;
typedef Eigen::Matrix<int, 2, 7> Mat27i;
typedef Eigen::Matrix<int, 3, 7> Mat37i;
typedef Eigen::Matrix<int, 4, 7> Mat47i;
typedef Eigen::Matrix<int, 5, 7> Mat57i;
typedef Eigen::Matrix<int, 6, 7> Mat67i;

typedef Eigen::Matrix<int, 6, 6> Mat66i;		//6
typedef Eigen::Matrix<int, 6, 5> Mat65i;
typedef Eigen::Matrix<int, 6, 4> Mat64i;
typedef Eigen::Matrix<int, 6, 3> Mat63i;
typedef Eigen::Matrix<int, 6, 2> Mat62i;
typedef Eigen::Matrix<int, 2, 6> Mat26i;
typedef Eigen::Matrix<int, 3, 6> Mat36i;
typedef Eigen::Matrix<int, 4, 6> Mat46i;
typedef Eigen::Matrix<int, 5, 6> Mat56i;

typedef Eigen::Matrix<int, 5, 5> Mat55i;		//5
typedef Eigen::Matrix<int, 5, 4> Mat54i;
typedef Eigen::Matrix<int, 5, 3> Mat53i;
typedef Eigen::Matrix<int, 5, 2> Mat52i;
typedef Eigen::Matrix<int, 2, 5> Mat25i;
typedef Eigen::Matrix<int, 3, 5> Mat35i;
typedef Eigen::Matrix<int, 4, 5> Mat45i;

typedef Eigen::Matrix<int, 4, 4> Mat44i;		//4
typedef Eigen::Matrix<int, 4, 3> Mat43i;
typedef Eigen::Matrix<int, 4, 2> Mat42i;
typedef Eigen::Matrix<int, 2, 4> Mat24i;
typedef Eigen::Matrix<int, 3, 4> Mat34i;
typedef Eigen::Matrix<int, 4, 4> Mat44i;

typedef Eigen::Matrix<int, 3, 3> Mat33i;		//3
typedef Eigen::Matrix<int, 3, 2> Mat32i;
typedef Eigen::Matrix<int, 2, 3> Mat23i;

typedef Eigen::Matrix<int, 2, 2> Mat22i;		//2

// int vectors
typedef Eigen::Matrix<int, Eigen::Dynamic, 1> VecXi;

typedef Eigen::Matrix<int, 10, 1> Vec10i;
typedef Eigen::Matrix<int, 9, 1> Vec9i;
typedef Eigen::Matrix<int, 8, 1> Vec8i;
typedef Eigen::Matrix<int, 7, 1> Vec7i;
typedef Eigen::Matrix<int, 6, 1> Vec6i;
typedef Eigen::Matrix<int, 5, 1> Vec5i;
typedef Eigen::Matrix<int, 4, 1> Vec4i;
typedef Eigen::Matrix<int, 3, 1> Vec3i;
typedef Eigen::Matrix<int, 2, 1> Vec2i;

// float matrices
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MatXXf;

typedef Eigen::Matrix<float, 10, 10> Mat1010f;		//10
typedef Eigen::Matrix<float, 10, 9> Mat109f;
typedef Eigen::Matrix<float, 10, 8> Mat108f;
typedef Eigen::Matrix<float, 10, 7> Mat107f;
typedef Eigen::Matrix<float, 10, 6> Mat106f;
typedef Eigen::Matrix<float, 10, 5> Mat105f;
typedef Eigen::Matrix<float, 10, 4> Mat104f;
typedef Eigen::Matrix<float, 10, 3> Mat103f;
typedef Eigen::Matrix<float, 10, 2> Mat102f;
typedef Eigen::Matrix<float, 2, 10> Mat210f;
typedef Eigen::Matrix<float, 3, 10> Mat310f;
typedef Eigen::Matrix<float, 4, 10> Mat410f;
typedef Eigen::Matrix<float, 5, 10> Mat510f;
typedef Eigen::Matrix<float, 6, 10> Mat610f;
typedef Eigen::Matrix<float, 7, 10> Mat710f;
typedef Eigen::Matrix<float, 8, 10> Mat810f;
typedef Eigen::Matrix<float, 9, 10> Mat910f;

typedef Eigen::Matrix<float, 9, 9> Mat99f;		//9
typedef Eigen::Matrix<float, 9, 8> Mat98f;
typedef Eigen::Matrix<float, 9, 7> Mat97f;
typedef Eigen::Matrix<float, 9, 6> Mat96f;
typedef Eigen::Matrix<float, 9, 5> Mat95f;
typedef Eigen::Matrix<float, 9, 4> Mat94f;
typedef Eigen::Matrix<float, 9, 3> Mat93f;
typedef Eigen::Matrix<float, 9, 2> Mat92f;
typedef Eigen::Matrix<float, 2, 9> Mat29f;
typedef Eigen::Matrix<float, 3, 9> Mat39f;
typedef Eigen::Matrix<float, 4, 9> Mat49f;
typedef Eigen::Matrix<float, 5, 9> Mat59f;
typedef Eigen::Matrix<float, 6, 9> Mat69f;
typedef Eigen::Matrix<float, 7, 9> Mat79f;
typedef Eigen::Matrix<float, 8, 9> Mat89f;

typedef Eigen::Matrix<float, 8, 8> Mat88f;		//8
typedef Eigen::Matrix<float, 8, 7> Mat87f;
typedef Eigen::Matrix<float, 8, 6> Mat86f;
typedef Eigen::Matrix<float, 8, 5> Mat85f;
typedef Eigen::Matrix<float, 8, 4> Mat84f;
typedef Eigen::Matrix<float, 8, 3> Mat83f;
typedef Eigen::Matrix<float, 8, 2> Mat82f;
typedef Eigen::Matrix<float, 2, 8> Mat28f;
typedef Eigen::Matrix<float, 3, 8> Mat38f;
typedef Eigen::Matrix<float, 4, 8> Mat48f;
typedef Eigen::Matrix<float, 5, 8> Mat58f;
typedef Eigen::Matrix<float, 6, 8> Mat68f;
typedef Eigen::Matrix<float, 7, 8> Mat78f;

typedef Eigen::Matrix<float, 7, 7> Mat77f;		//7
typedef Eigen::Matrix<float, 7, 6> Mat76f;
typedef Eigen::Matrix<float, 7, 5> Mat75f;
typedef Eigen::Matrix<float, 7, 4> Mat74f;
typedef Eigen::Matrix<float, 7, 3> Mat73f;
typedef Eigen::Matrix<float, 7, 2> Mat72f;
typedef Eigen::Matrix<float, 2, 7> Mat27f;
typedef Eigen::Matrix<float, 3, 7> Mat37f;
typedef Eigen::Matrix<float, 4, 7> Mat47f;
typedef Eigen::Matrix<float, 5, 7> Mat57f;
typedef Eigen::Matrix<float, 6, 7> Mat67f;

typedef Eigen::Matrix<float, 6, 6> Mat66f;		//6
typedef Eigen::Matrix<float, 6, 5> Mat65f;
typedef Eigen::Matrix<float, 6, 4> Mat64f;
typedef Eigen::Matrix<float, 6, 3> Mat63f;
typedef Eigen::Matrix<float, 6, 2> Mat62f;
typedef Eigen::Matrix<float, 2, 6> Mat26f;
typedef Eigen::Matrix<float, 3, 6> Mat36f;
typedef Eigen::Matrix<float, 4, 6> Mat46f;
typedef Eigen::Matrix<float, 5, 6> Mat56f;

typedef Eigen::Matrix<float, 5, 5> Mat55f;		//5
typedef Eigen::Matrix<float, 5, 4> Mat54f;
typedef Eigen::Matrix<float, 5, 3> Mat53f;
typedef Eigen::Matrix<float, 5, 2> Mat52f;
typedef Eigen::Matrix<float, 2, 5> Mat25f;
typedef Eigen::Matrix<float, 3, 5> Mat35f;
typedef Eigen::Matrix<float, 4, 5> Mat45f;

typedef Eigen::Matrix<float, 4, 4> Mat44f;		//4
typedef Eigen::Matrix<float, 4, 3> Mat43f;
typedef Eigen::Matrix<float, 4, 2> Mat42f;
typedef Eigen::Matrix<float, 2, 4> Mat24f;
typedef Eigen::Matrix<float, 3, 4> Mat34f;
typedef Eigen::Matrix<float, 4, 4> Mat44f;

typedef Eigen::Matrix<float, 3, 3> Mat33f;		//3
typedef Eigen::Matrix<float, 3, 2> Mat32f;
typedef Eigen::Matrix<float, 2, 3> Mat23f;

typedef Eigen::Matrix<float, 2, 2> Mat22f;		//2

// float vectors
typedef Eigen::Matrix<float, Eigen::Dynamic, 1> VecXf;

typedef Eigen::Matrix<float, 10, 1> Vec10f;
typedef Eigen::Matrix<float, 9, 1> Vec9f;
typedef Eigen::Matrix<float, 8, 1> Vec8f;
typedef Eigen::Matrix<float, 7, 1> Vec7f;
typedef Eigen::Matrix<float, 6, 1> Vec6f;
typedef Eigen::Matrix<float, 5, 1> Vec5f;
typedef Eigen::Matrix<float, 4, 1> Vec4f;
typedef Eigen::Matrix<float, 3, 1> Vec3f;
typedef Eigen::Matrix<float, 2, 1> Vec2f;

// double matrices
typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> MatXXd;

typedef Eigen::Matrix<double, 10, 10> Mat1010d;		//10
typedef Eigen::Matrix<double, 10, 9> Mat109d;
typedef Eigen::Matrix<double, 10, 8> Mat108d;
typedef Eigen::Matrix<double, 10, 7> Mat107d;
typedef Eigen::Matrix<double, 10, 6> Mat106d;
typedef Eigen::Matrix<double, 10, 5> Mat105d;
typedef Eigen::Matrix<double, 10, 4> Mat104d;
typedef Eigen::Matrix<double, 10, 3> Mat103d;
typedef Eigen::Matrix<double, 10, 2> Mat102d;
typedef Eigen::Matrix<double, 2, 10> Mat210d;
typedef Eigen::Matrix<double, 3, 10> Mat310d;
typedef Eigen::Matrix<double, 4, 10> Mat410d;
typedef Eigen::Matrix<double, 5, 10> Mat510d;
typedef Eigen::Matrix<double, 6, 10> Mat610d;
typedef Eigen::Matrix<double, 7, 10> Mat710d;
typedef Eigen::Matrix<double, 8, 10> Mat810d;
typedef Eigen::Matrix<double, 9, 10> Mat910d;

typedef Eigen::Matrix<double, 9, 9> Mat99d;		//9
typedef Eigen::Matrix<double, 9, 8> Mat98d;
typedef Eigen::Matrix<double, 9, 7> Mat97d;
typedef Eigen::Matrix<double, 9, 6> Mat96d;
typedef Eigen::Matrix<double, 9, 5> Mat95d;
typedef Eigen::Matrix<double, 9, 4> Mat94d;
typedef Eigen::Matrix<double, 9, 3> Mat93d;
typedef Eigen::Matrix<double, 9, 2> Mat92d;
typedef Eigen::Matrix<double, 2, 9> Mat29d;
typedef Eigen::Matrix<double, 3, 9> Mat39d;
typedef Eigen::Matrix<double, 4, 9> Mat49d;
typedef Eigen::Matrix<double, 5, 9> Mat59d;
typedef Eigen::Matrix<double, 6, 9> Mat69d;
typedef Eigen::Matrix<double, 7, 9> Mat79d;
typedef Eigen::Matrix<double, 8, 9> Mat89d;

typedef Eigen::Matrix<double, 8, 8> Mat88d;		//8
typedef Eigen::Matrix<double, 8, 7> Mat87d;
typedef Eigen::Matrix<double, 8, 6> Mat86d;
typedef Eigen::Matrix<double, 8, 5> Mat85d;
typedef Eigen::Matrix<double, 8, 4> Mat84d;
typedef Eigen::Matrix<double, 8, 3> Mat83d;
typedef Eigen::Matrix<double, 8, 2> Mat82d;
typedef Eigen::Matrix<double, 2, 8> Mat28d;
typedef Eigen::Matrix<double, 3, 8> Mat38d;
typedef Eigen::Matrix<double, 4, 8> Mat48d;
typedef Eigen::Matrix<double, 5, 8> Mat58d;
typedef Eigen::Matrix<double, 6, 8> Mat68d;
typedef Eigen::Matrix<double, 7, 8> Mat78d;

typedef Eigen::Matrix<double, 7, 7> Mat77d;		//7
typedef Eigen::Matrix<double, 7, 6> Mat76d;
typedef Eigen::Matrix<double, 7, 5> Mat75d;
typedef Eigen::Matrix<double, 7, 4> Mat74d;
typedef Eigen::Matrix<double, 7, 3> Mat73d;
typedef Eigen::Matrix<double, 7, 2> Mat72d;
typedef Eigen::Matrix<double, 2, 7> Mat27d;
typedef Eigen::Matrix<double, 3, 7> Mat37d;
typedef Eigen::Matrix<double, 4, 7> Mat47d;
typedef Eigen::Matrix<double, 5, 7> Mat57d;
typedef Eigen::Matrix<double, 6, 7> Mat67d;

typedef Eigen::Matrix<double, 6, 6> Mat66d;		//6
typedef Eigen::Matrix<double, 6, 5> Mat65d;
typedef Eigen::Matrix<double, 6, 4> Mat64d;
typedef Eigen::Matrix<double, 6, 3> Mat63d;
typedef Eigen::Matrix<double, 6, 2> Mat62d;
typedef Eigen::Matrix<double, 2, 6> Mat26d;
typedef Eigen::Matrix<double, 3, 6> Mat36d;
typedef Eigen::Matrix<double, 4, 6> Mat46d;
typedef Eigen::Matrix<double, 5, 6> Mat56d;

typedef Eigen::Matrix<double, 5, 5> Mat55d;		//5
typedef Eigen::Matrix<double, 5, 4> Mat54d;
typedef Eigen::Matrix<double, 5, 3> Mat53d;
typedef Eigen::Matrix<double, 5, 2> Mat52d;
typedef Eigen::Matrix<double, 2, 5> Mat25d;
typedef Eigen::Matrix<double, 3, 5> Mat35d;
typedef Eigen::Matrix<double, 4, 5> Mat45d;

typedef Eigen::Matrix<double, 4, 4> Mat44d;		//4
typedef Eigen::Matrix<double, 4, 3> Mat43d;
typedef Eigen::Matrix<double, 4, 2> Mat42d;
typedef Eigen::Matrix<double, 2, 4> Mat24d;
typedef Eigen::Matrix<double, 3, 4> Mat34d;
typedef Eigen::Matrix<double, 4, 4> Mat44d;

typedef Eigen::Matrix<double, 3, 3> Mat33d;		//3
typedef Eigen::Matrix<double, 3, 2> Mat32d;
typedef Eigen::Matrix<double, 2, 3> Mat23d;

typedef Eigen::Matrix<double, 2, 2> Mat22d;		//2

// double vectors
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VecXd;

typedef Eigen::Matrix<double, 15, 1> Vec15d;
typedef Eigen::Matrix<double, 10, 1> Vec10d;
typedef Eigen::Matrix<double, 9, 1> Vec9d;
typedef Eigen::Matrix<double, 8, 1> Vec8d;
typedef Eigen::Matrix<double, 7, 1> Vec7d;
typedef Eigen::Matrix<double, 6, 1> Vec6d;
typedef Eigen::Matrix<double, 5, 1> Vec5d;
typedef Eigen::Matrix<double, 4, 1> Vec4d;
typedef Eigen::Matrix<double, 3, 1> Vec3d;
typedef Eigen::Matrix<double, 2, 1> Vec2d;

}

