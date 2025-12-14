import axios from 'axios';

// API Configuration - Hugging Face Spaces Backend
const API_BASE_URL = 'https://shurem-ragchatbot.hf.space/v1';

// Create axios instance with default config
const apiClient = axios.create({
  baseURL: API_BASE_URL,
  timeout: 30000, // 30 seconds timeout
  headers: {
    'Content-Type': 'application/json',
  },
});

// Add request interceptor to include auth token
apiClient.interceptors.request.use(
  (config) => {
    const token = localStorage.getItem('api_token'); // Or however you store the token
    if (token) {
      config.headers.Authorization = `Bearer ${token}`;
    }
    return config;
  },
  (error) => {
    return Promise.reject(error);
  }
);

// Add response interceptor for error handling
apiClient.interceptors.response.use(
  (response) => response,
  (error) => {
    console.error('API Error:', error);
    return Promise.reject(error);
  }
);

// Types
export interface Conversation {
  conversation_id: string;
  created_at: string;
  title: string;
  is_active: boolean;
  messages?: Message[];
}

export interface Message {
  message_id: string;
  conversation_id: string;
  sender_type: 'user' | 'system';
  content: string;
  created_at: string;
  citations?: Citation[];
  message_type: string;
}

export interface Citation {
  chapter_title: string;
  section_title: string;
  url_path: string;
  confidence_score: number;
  content_snippet: string;
}

export interface CreateConversationRequest {
  user_id?: string;
  session_id?: string;
  initial_query?: string;
}

export interface CreateMessageRequest {
  content: string;
  message_type?: 'query' | 'follow-up' | 'text-selection';
}

export interface SearchRequest {
  query: string;
  max_results?: number;
  include_citations?: boolean;
}

export interface SearchResponse {
  query: string;
  results: SearchResult[];
  total_results: number;
}

export interface SearchResult {
  content_id: string;
  chapter_title: string;
  section_title: string;
  url_path: string;
  content_snippet: string;
  relevance_score: number;
}

// API functions
export const chatbotApi = {
  // Conversation management
  createConversation: async (data: CreateConversationRequest): Promise<Conversation> => {
    const response = await apiClient.post<Conversation>('/conversations', data);
    return response.data;
  },

  getConversation: async (conversationId: string): Promise<Conversation> => {
    const response = await apiClient.get<Conversation>(`/conversations/${conversationId}`);
    return response.data;
  },

  clearConversation: async (conversationId: string): Promise<void> => {
    await apiClient.delete(`/conversations/${conversationId}/messages`);
  },

  // Messaging
  sendMessage: async (conversationId: string, data: CreateMessageRequest): Promise<Message> => {
    const response = await apiClient.post<Message>(`/conversations/${conversationId}/messages`, data);
    return response.data;
  },

  // Citations
  getMessageCitations: async (messageId: string): Promise<{ message_id: string; citations: Citation[] }> => {
    const response = await apiClient.get<{ message_id: string; citations: Citation[] }>(`/messages/${messageId}/citations`);
    return response.data;
  },

  // Search
  searchContent: async (data: SearchRequest): Promise<SearchResponse> => {
    const response = await apiClient.post<SearchResponse>('/search', data);
    return response.data;
  },

  // Health check
  healthCheck: async (): Promise<{ status: string; checks: Record<string, string> }> => {
    const response = await apiClient.get('/health');
    return response.data;
  },
};

// SSE Streaming function for real-time responses
export const streamMessage = async (
  conversationId: string,
  data: CreateMessageRequest,
  onMessage: (data: any) => void,
  onError?: (error: any) => void
): Promise<void> => {
  const token = localStorage.getItem('api_token') || '';
  const response = await fetch(
    `${API_BASE_URL}/conversations/${conversationId}/messages`,
    {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${token}`,
        'Accept': 'text/event-stream',
        'Cache-Control': 'no-cache',
      },
      body: JSON.stringify(data),
    }
  );

  if (!response.ok) {
    throw new Error(`HTTP error! status: ${response.status}`);
  }

  const reader = response.body?.getReader();
  const decoder = new TextDecoder();

  if (!reader) {
    throw new Error('No response body reader available');
  }

  try {
    while (true) {
      const { done, value } = await reader.read();
      if (done) break;

      const chunk = decoder.decode(value);
      const lines = chunk.split('\n');

      for (const line of lines) {
        if (line.startsWith('data: ')) {
          try {
            const data = JSON.parse(line.slice(6)); // Remove 'data: ' prefix
            onMessage(data);
          } catch (e) {
            console.error('Error parsing SSE data:', line, e);
          }
        }
      }
    }
  } finally {
    reader.releaseLock();
  }
};

export default apiClient;