#pragma once

#include <algorithm>
#include <fmt/core.h>
#include <functional>
#include <future>
#include <iomanip>
#include <sstream>
#include <utility>
#include <vector>

namespace utils {

template<class T>
using Ref = std::reference_wrapper<T>;

template<class T>
using CRef = std::reference_wrapper<const T>;

template<class T>
using RefVec = std::vector<Ref<T>>;

template<class T>
using CRefVec = std::vector<CRef<T>>;

template<class T>
struct is_ref : std::false_type
{
};
template<class T>
struct is_ref<Ref<T>> : std::true_type
{
};
template<class T>
struct is_ref<CRef<T>> : std::true_type
{
};

template<class T>
constexpr auto is_ref_v = is_ref<T>::value;

template<class Head0, class Head1, class... Tail>
constexpr auto min(Head0&& head0, Head1&& head1, Tail&&... tail) noexcept
{
    if constexpr(sizeof...(tail) == 0) {
        return head0 < head1 ? head0 : head1;
    } else {
        return min(
            min(std::forward<Head0>(head0),
                std::forward<Head1>(head1)),
            std::forward<Tail>(tail)...);
    }
}

template<class Head0, class Head1, class... Tail>
constexpr auto max(Head0&& head0, Head1&& head1, Tail&&... tail) noexcept
{
    if constexpr(sizeof...(tail) == 0) {
        return head0 > head1 ? head0 : head1;
    } else {
        return max(
            max(std::forward<Head0>(head0),
                std::forward<Head1>(head1)),
            std::forward<Tail>(tail)...);
    }
}

template<class Head0, class Head1, class... Tail>
constexpr auto concat(Head0&& head0, Head1&& head1, Tail&&... tail) noexcept
{
    if constexpr(sizeof...(tail) == 0) {
        head0.insert(std::end(head0),
                     std::begin(head1),
                     std::end(head1));
        return std::forward<Head0>(head0);
    } else {
        return concat(
            concat(std::forward<Head0>(head0),
                   std::forward<Head1>(head1)),
            std::forward<Tail>(tail)...);
    }
}

template<class T>
auto collectFutures(std::vector<std::future<T>>&& futures) noexcept
    -> std::vector<T>
{
    std::vector<T> collected;
    collected.reserve(futures.size());

    std::transform(std::make_move_iterator(std::begin(futures)),
                   std::make_move_iterator(std::end(futures)),
                   std::back_inserter(collected),
                   [](auto future) {
                       return future.get();
                   });

    return collected;
}


template<class Head0, class Head1, class... Tail>
auto await(std::future<Head0>&& head0,
           std::future<Head1>&& head1,
           std::future<Tail>&&... tail) noexcept
    -> std::tuple<Head0, Head1, Tail...>
{
    return std::tuple{head0.get(),
                      head1.get(),
                      tail.get()...};
}

template<class T, class... Tail>
auto intersect(std::vector<T>&& head0,
               std::vector<T>&& head1,
               Tail&&... tail) noexcept
    -> std::vector<T>
{
    if constexpr(sizeof...(tail) == 0) {
        std::vector<T> intersection;
        std::set_intersection(std::make_move_iterator(std::begin(head0)),
                              std::make_move_iterator(std::end(head0)),
                              std::make_move_iterator(std::begin(head1)),
                              std::make_move_iterator(std::end(head1)),
                              std::back_inserter(intersection));
        return intersection;
    } else {
        return intersect(
            intersect(std::move(head0),
                      std::move(head1)),
            std::forward<Tail>(tail)...);
    }
}

template<class T, class... Tail>
auto intersect(const std::vector<T>& head0,
               const std::vector<T>& head1,
               Tail&&... tail) noexcept
    -> std::vector<T>
{
    if constexpr(sizeof...(tail) == 0) {
        std::vector<T> intersection;
        std::set_intersection(std::begin(head0),
                              std::end(head0),
                              std::begin(head1),
                              std::end(head1),
                              std::back_inserter(intersection));
        return intersection;
    } else {
        return intersect(
            intersect(head0,
                      head1),
            std::forward<Tail>(tail)...);
    }
}

template<class T, class F, class... Tail>
auto intersect(std::vector<T>&& head0,
               std::vector<T>&& head1,
               Tail&&... tail,
               F&& func) noexcept
    -> std::vector<T>
{
    if constexpr(sizeof...(tail) == 0) {
        std::vector<T> intersection;
        std::set_intersection(std::make_move_iterator(std::begin(head0)),
                              std::make_move_iterator(std::end(head0)),
                              std::make_move_iterator(std::begin(head1)),
                              std::make_move_iterator(std::end(head1)),
                              std::back_inserter(intersection),
                              std::forward<F>(func));
        return intersection;
    } else {
        return intersect(
            intersect(std::move(head0),
                      std::move(head1),
                      std::forward<F>(func)),
            std::forward<Tail>(tail)...,
            std::forward<F>(func));
    }
}

template<class T, class F, class... Tail>
auto intersect(const std::vector<T>& head0,
               const std::vector<T>& head1,
               Tail&&... tail,
               F&& func) noexcept
    -> std::vector<T>
{
    if constexpr(sizeof...(tail) == 0) {
        std::vector<T> intersection;
        std::set_intersection(std::begin(head0),
                              std::end(head0),
                              std::begin(head1),
                              std::end(head1),
                              std::back_inserter(intersection),
                              std::forward<F>(func));
        return intersection;
    } else {
        return intersect(
            intersect(head0,
                      head1,
                      std::forward<F>(func)),
            std::forward<Tail>(tail)...,
            std::forward<F>(func));
    }
}

template<class Head0, class Head1, class... Tail>
auto hashCombine(Head0&& head0, Head1&& head1, Tail&&... tail)
{
    if constexpr(sizeof...(tail) == 0) {
        std::hash<std::remove_const_t<std::remove_reference_t<Head0>>> hasher0;
        std::hash<std::remove_const_t<std::remove_reference_t<Head1>>> hasher1;
        auto seed = hasher0(head0);
        return seed xor (hasher1(head1) + 0x9e3779b9 + (seed << 6) + (seed >> 2));
    } else {
        return hashCombine(
            hashCombine(head0, head1),
            std::forward<Tail>(tail)...);
    }
}


inline auto unquote(const std::string& s)
    -> std::string
{
    std::string result;
    std::istringstream ss(s);
    ss >> std::quoted(result);
    return result;
}

template<class... Ts>
struct Overloaded : Ts...
{
    using Ts::operator()...;
};

template<class... Ts>
Overloaded(Ts...) -> Overloaded<Ts...>;


} // namespace utils
