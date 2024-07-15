#include <iostream>
#include <string>
#include <curl/curl.h>
#include <nlohmann/json.hpp>

std::string apiKey = "678035a1c2mshd447467226dc590p1afa71jsn5f0f21fdad45";
std::string apiHost = "chess-stockfish-16-api.p.rapidapi.com";

using Json = nlohmann::json;

// Callback function to handle data received from CURL request
size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

std::string getBestMove(const std::string& fen) {
    CURL* curl;
    CURLcode res;
    std::string readBuffer;

    curl_global_init(CURL_GLOBAL_DEFAULT);
    curl = curl_easy_init();

    std::string bestMove = "Error"; // Default error message

    if(curl) {
        struct curl_slist* headers = NULL;
        headers = curl_slist_append(headers, ("X-RapidAPI-Key: " + apiKey).c_str());
        headers = curl_slist_append(headers, ("X-RapidAPI-Host: " + apiHost).c_str());

        std::string url = "https://chess-stockfish-16-api.p.rapidapi.com/bestmove";
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);

        // Set the POST fields
        Json jsonData;
        jsonData["fen"] = fen;
        jsonData["depth"] = 15;  // You can set the desired depth here

        std::string postData = jsonData.dump();

        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, postData.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, postData.size());

        // Perform the request, res will get the return code
        res = curl_easy_perform(curl);

        // Check for errors
        if(res != CURLE_OK) {
            std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
        }

        // Clean up
        curl_easy_cleanup(curl);
        curl_slist_free_all(headers);
    }

    curl_global_cleanup();

    // Parse the response using nlohmann/json
    try {
        auto jsonResponse = nlohmann::json::parse(readBuffer);
        bestMove = jsonResponse["bestMove"].get<std::string>();
    } catch (nlohmann::json::parse_error& e) {
        std::cerr << "Failed to parse the response: " << e.what() << std::endl;
    }

    return bestMove;
}
