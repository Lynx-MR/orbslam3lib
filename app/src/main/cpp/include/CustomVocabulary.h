#ifndef CUSTOM_VOCABULARY_H
#define CUSTOM_VOCABULARY_H

#include "DBoW2/TemplatedVocabulary.h"
#include <fstream>

namespace ORB_SLAM3 {

    class CustomVocabulary : public DBoW2::TemplatedVocabulary<cv::Mat, DBoW2::FORB> {
    public:
        CustomVocabulary() : TemplatedVocabulary() {}

        /*void loadBinary(const std::string& filename) {
            std::ifstream f(filename, std::ios::binary);
            if (!f.is_open()) {
                throw std::runtime_error("Could not open binary vocab file: " + filename);
            }

            // Accès direct aux membres protégés (m_k, m_L, etc.)
            f.read((char*)&m_k, sizeof(m_k));
            f.read((char*)&m_L, sizeof(m_L));
            f.read((char*)&m_scoring, sizeof(m_scoring));
            f.read((char*)&m_weighting, sizeof(m_weighting));
            createScoringObject();

            // Lire les noeuds
            size_t num_nodes;
            f.read((char*)&num_nodes, sizeof(num_nodes));
            m_nodes.resize(num_nodes);

            for (size_t i = 0; i < num_nodes; ++i) {
                Node& node = m_nodes[i];
                f.read((char*)&node.id, sizeof(node.id));
                f.read((char*)&node.weight, sizeof(node.weight));
                f.read((char*)&node.parent, sizeof(node.parent));
                f.read((char*)&node.word_id, sizeof(node.word_id));

                // Lire les enfants
                size_t num_children;
                f.read((char*)&num_children, sizeof(num_children));
                node.children.resize(num_children);
                f.read((char*)node.children.data(), num_children * sizeof(DBoW2::NodeId));

                // Lire le descripteur (cv::Mat)
                int rows, cols, type;
                f.read((char*)&rows, sizeof(rows));
                f.read((char*)&cols, sizeof(cols));
                f.read((char*)&type, sizeof(type));
                node.descriptor.create(rows, cols, type);
                f.read((char*)node.descriptor.data, node.descriptor.total() * node.descriptor.elemSize());
            }

            // Reconstruire les mots et poids
            createWords();
            std::vector<std::vector<cv::Mat>> dummy_features; // Ajuste si besoin
            setNodeWeights(dummy_features);
            f.close();
        }*/

        void loadBinary(const std::string& filename) {
            fstream f;
            f.open(filename.c_str(), ios_base::in|ios::binary);
            unsigned int nb_nodes, size_node;
            f.read((char*)&nb_nodes, sizeof(nb_nodes));
            f.read((char*)&size_node, sizeof(size_node));
            f.read((char*)&m_k, sizeof(m_k));
            f.read((char*)&m_L, sizeof(m_L));
            f.read((char*)&m_scoring, sizeof(m_scoring));
            f.read((char*)&m_weighting, sizeof(m_weighting));
            createScoringObject();

            m_words.clear();
            m_words.reserve(pow((double)m_k, (double)m_L + 1));
            m_nodes.clear();
            m_nodes.resize(nb_nodes+1);
            m_nodes[0].id = 0;
            char buf[size_node]; int nid = 1;
            while (!f.eof()) {
                f.read(buf, size_node);
                m_nodes[nid].id = nid;
                // FIXME
                const int* ptr=(int*)buf;
                m_nodes[nid].parent = *ptr;
                //m_nodes[nid].parent = *(const int*)buf;
                m_nodes[m_nodes[nid].parent].children.push_back(nid);
                m_nodes[nid].descriptor = cv::Mat(1, DBoW2::FORB::L, CV_8U);
                memcpy(m_nodes[nid].descriptor.data, buf+4, DBoW2::FORB::L);
                m_nodes[nid].weight = *(float*)(buf+4+DBoW2::FORB::L);
                if (buf[8+DBoW2::FORB::L]) { // is leaf
                    int wid = m_words.size();
                    m_words.resize(wid+1);
                    m_nodes[nid].word_id = wid;
                    m_words[wid] = &m_nodes[nid];
                }
                else
                    m_nodes[nid].children.reserve(m_k);
                nid+=1;
            }
            f.close();
        }
    };

} // namespace ORB_SLAM3

#endif