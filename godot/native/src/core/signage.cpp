#include "signage.hpp"
#include <algorithm>

namespace ec {

bool cyrillic_region(double lat, double lon) { return lon > 20 && lon < 62 && lat > 40 && lat < 72; }

static std::string lower_ascii(std::string s) {
    for (auto &c : s)
        if (c >= 'A' && c <= 'Z') c = char(c + 32);
    return s;
}

struct Row {
    const char *value;
    float br, bg, bb;  // board
    float tr, tg, tb;  // text
    int awning, icon;
    const char *ru, *en;
};

static const Row ROWS[] = {
    {"supermarket", 0.10f, 0.48f, 0.22f, 1, 1, 1, 0, 0, "Супермаркет", "Supermarket"},
    {"convenience", 0.85f, 0.42f, 0.10f, 1, 1, 1, 1, 0, "Продукты", "Convenience"},
    {"grocery", 0.85f, 0.42f, 0.10f, 1, 1, 1, 1, 0, "Продукты", "Grocery"},
    {"greengrocer", 0.35f, 0.62f, 0.15f, 1, 1, 1, 1, 0, "Овощи и фрукты", "Greengrocer"},
    {"butcher", 0.65f, 0.12f, 0.12f, 1, 1, 1, 1, 0, "Мясо", "Butcher"},
    {"bakery", 0.55f, 0.33f, 0.15f, 1, 0.95f, 0.8f, 1, 0, "Хлеб", "Bakery"},
    {"confectionery", 0.8f, 0.4f, 0.5f, 1, 1, 1, 1, 0, "Кондитерская", "Confectionery"},
    {"alcohol", 0.35f, 0.1f, 0.2f, 1, 0.9f, 0.7f, 0, 0, "Алкоголь", "Alcohol"},
    {"pharmacy", 0.08f, 0.55f, 0.30f, 1, 1, 1, 0, 1, "Аптека", "Pharmacy"},
    {"chemist", 0.35f, 0.65f, 0.75f, 1, 1, 1, 0, 0, "Бытовая химия", "Drugstore"},
    {"clothes", 0.12f, 0.12f, 0.14f, 1, 1, 1, 0, 0, "Одежда", "Clothes"},
    {"shoes", 0.30f, 0.20f, 0.14f, 1, 1, 1, 0, 0, "Обувь", "Shoes"},
    {"jewelry", 0.1f, 0.1f, 0.18f, 0.95f, 0.8f, 0.35f, 0, 0, "Ювелирный", "Jewelry"},
    {"hairdresser", 0.45f, 0.2f, 0.5f, 1, 1, 1, 0, 3, "Парикмахерская", "Hair salon"},
    {"beauty", 0.6f, 0.25f, 0.45f, 1, 1, 1, 0, 0, "Салон красоты", "Beauty"},
    {"florist", 0.55f, 0.15f, 0.35f, 1, 1, 1, 1, 0, "Цветы", "Flowers"},
    {"books", 0.2f, 0.28f, 0.5f, 1, 1, 1, 0, 0, "Книги", "Books"},
    {"stationery", 0.2f, 0.4f, 0.6f, 1, 1, 1, 0, 0, "Канцтовары", "Stationery"},
    {"mobile_phone", 0.1f, 0.3f, 0.65f, 1, 1, 1, 0, 0, "Телефоны", "Phones"},
    {"electronics", 0.1f, 0.3f, 0.65f, 1, 1, 1, 0, 0, "Электроника", "Electronics"},
    {"computer", 0.1f, 0.3f, 0.65f, 1, 1, 1, 0, 0, "Компьютеры", "Computers"},
    {"hardware", 0.85f, 0.45f, 0.08f, 0.1f, 0.1f, 0.1f, 0, 0, "Хозтовары", "Hardware"},
    {"doityourself", 0.85f, 0.45f, 0.08f, 0.1f, 0.1f, 0.1f, 0, 0, "Строительные товары", "DIY"},
    {"furniture", 0.4f, 0.3f, 0.2f, 1, 1, 1, 0, 0, "Мебель", "Furniture"},
    {"optician", 0.15f, 0.45f, 0.6f, 1, 1, 1, 0, 0, "Оптика", "Optician"},
    {"toys", 0.9f, 0.7f, 0.1f, 0.15f, 0.15f, 0.4f, 0, 0, "Игрушки", "Toys"},
    {"gift", 0.7f, 0.2f, 0.3f, 1, 1, 1, 0, 0, "Подарки", "Gifts"},
    {"sports", 0.1f, 0.4f, 0.2f, 1, 1, 1, 0, 0, "Спорттовары", "Sports"},
    {"car_repair", 0.2f, 0.22f, 0.25f, 1, 0.8f, 0.1f, 0, 0, "Автосервис", "Car repair"},
    {"car_parts", 0.2f, 0.22f, 0.25f, 1, 0.8f, 0.1f, 0, 0, "Автозапчасти", "Car parts"},
    {"mall", 0.2f, 0.2f, 0.25f, 1, 1, 1, 0, 0, "Торговый центр", "Mall"},
    {"department_store", 0.2f, 0.2f, 0.25f, 1, 1, 1, 0, 0, "Универмаг", "Department store"},
    {"kiosk", 0.3f, 0.5f, 0.35f, 1, 1, 1, 1, 0, "Киоск", "Kiosk"},
    {"cafe", 0.40f, 0.24f, 0.12f, 1, 0.95f, 0.85f, 1, 2, "Кафе", "Cafe"},
    {"restaurant", 0.45f, 0.08f, 0.10f, 1, 0.95f, 0.85f, 1, 0, "Ресторан", "Restaurant"},
    {"fast_food", 0.8f, 0.12f, 0.08f, 1, 0.9f, 0.2f, 1, 0, "Фастфуд", "Fast food"},
    {"bar", 0.12f, 0.1f, 0.14f, 0.9f, 0.7f, 0.3f, 0, 0, "Бар", "Bar"},
    {"pub", 0.12f, 0.1f, 0.14f, 0.9f, 0.7f, 0.3f, 0, 0, "Паб", "Pub"},
    {"bank", 0.0f, 0.42f, 0.36f, 1, 1, 1, 0, 0, "Банк", "Bank"},
    {"atm", 0.35f, 0.38f, 0.42f, 1, 1, 1, 0, 0, "Банкомат", "ATM"},
    {"post_office", 0.1f, 0.3f, 0.6f, 1, 1, 1, 0, 0, "Почта", "Post office"},
    {"clinic", 0.15f, 0.5f, 0.6f, 1, 1, 1, 0, 1, "Поликлиника", "Clinic"},
    {"dentist", 0.2f, 0.55f, 0.65f, 1, 1, 1, 0, 0, "Стоматология", "Dentist"},
    {"library", 0.3f, 0.25f, 0.2f, 1, 1, 1, 0, 0, "Библиотека", "Library"},
    {"cinema", 0.15f, 0.1f, 0.25f, 1, 0.95f, 0.75f, 0, 0, "Кинотеатр", "Cinema"},
    {"theatre", 0.3f, 0.08f, 0.12f, 1, 0.9f, 0.75f, 0, 0, "Театр", "Theatre"},
    {"marketplace", 0.35f, 0.5f, 0.2f, 1, 1, 1, 0, 0, "Рынок", "Market"},
    {"hotel", 0.15f, 0.2f, 0.35f, 1, 0.9f, 0.75f, 0, 0, "Гостиница", "Hotel"},
};

ShopStyle shop_style(const std::string &key, const std::string &value, const std::string &name, const std::string &brand, bool cyr) {
    ShopStyle st;
    st.board = {0.22f, 0.25f, 0.3f, 1};
    st.text = {1, 1, 1, 1};
    st.awning = {0.6f, 0.15f, 0.12f, 1};
    st.word = cyr ? (key == "office" ? "Офис" : "Магазин") : (key == "office" ? "Office" : "Shop");
    for (auto &r : ROWS)
        if (value == r.value) {
            st.board = {r.br, r.bg, r.bb, 1};
            st.text = {r.tr, r.tg, r.tb, 1};
            st.awning = {r.br * 0.9f, r.bg * 0.9f, r.bb * 0.9f, 1};
            st.awning_on = r.awning;
            st.icon = r.icon;
            st.word = cyr ? r.ru : r.en;
            break;
        }
    // a few well-known brands, by exact name or brand (colours approximate the real signage)
    struct Brand {
        const char *n;
        Col board, text;
    };
    static const Brand BR[] = {
        {"mcdonald's", {0.80f, 0.06f, 0.06f, 1}, {1.0f, 0.8f, 0.1f, 1}},
        {"макдоналдс", {0.80f, 0.06f, 0.06f, 1}, {1.0f, 0.8f, 0.1f, 1}},
        {"kfc", {0.75f, 0.05f, 0.1f, 1}, {1, 1, 1, 1}},
        {"subway", {0.0f, 0.5f, 0.2f, 1}, {1.0f, 0.85f, 0.1f, 1}},
        {"starbucks", {0.0f, 0.4f, 0.25f, 1}, {1, 1, 1, 1}},
        {"мтс", {0.85f, 0.05f, 0.1f, 1}, {1, 1, 1, 1}},
        {"mts", {0.85f, 0.05f, 0.1f, 1}, {1, 1, 1, 1}},
        {"a1", {0.85f, 0.1f, 0.1f, 1}, {1, 1, 1, 1}},
        {"а1", {0.85f, 0.1f, 0.1f, 1}, {1, 1, 1, 1}},
        {"евроопт", {0.75f, 0.05f, 0.1f, 1}, {1, 1, 1, 1}},
        {"evroopt", {0.75f, 0.05f, 0.1f, 1}, {1, 1, 1, 1}},
        {"белпочта", {0.1f, 0.3f, 0.6f, 1}, {1, 1, 1, 1}},
    };
    std::string n = lower_utf8(name), b = lower_utf8(brand);
    for (auto &x : BR)
        if (n == x.n || b == x.n) {
            st.board = x.board;
            st.text = x.text;
            break;
        }
    (void)lower_ascii;
    return st;
}

}  // namespace ec
